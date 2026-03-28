/*
 * rtlsrd-wsprd, WSPR daemon for RTL receivers
 * Copyright (C) 2016-2021, Guenael Jouchet (VA2GKA)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <getopt.h>
#include <math.h>
#include <string.h>
#include <sys/time.h>
#include <pthread.h>
#include <libhackrf/hackrf.h>
#include <curl/curl.h>

#include "./rtlsdr_wsprd.h"
#include "./wsprd/wsprd.h"
#include "./wsprd/wsprsim_utils.h"


/* Sampling definition for RTL devices & WSPR protocol */
#define SIGNAL_LENGHT       120
#define SIGNAL_SAMPLE_RATE  375
#define SAMPLING_RATE       2400000
#define FS4_RATE            SAMPLING_RATE / 4
#define DOWNSAMPLING        SAMPLING_RATE / SIGNAL_SAMPLE_RATE
#define DEFAULT_BUF_LENGTH  (4 * 16384)
#define FIR_TAPS            32


/* Debugging logs */
#define LOG_DEBUG   0
#define LOG_INFO    1
#define LOG_WARN    2
#define LOG_ERROR   3
#define LOG_LEVEL   LOG_ERROR
#define LOG(level, ...)  if (level >= LOG_LEVEL) fprintf(stderr, __VA_ARGS__)


#define safe_cond_signal(n, m) pthread_mutex_lock(m); pthread_cond_signal(n); pthread_mutex_unlock(m)
#define safe_cond_wait(n, m) pthread_mutex_lock(m); pthread_cond_wait(n, m); pthread_mutex_unlock(m)


/* Thread for decoding */
struct decoder_state {
    pthread_t        thread;
    pthread_attr_t   tattr;
    pthread_cond_t   ready_cond;
    pthread_mutex_t  ready_mutex;
};
static struct decoder_state decState;


/* Thread for RX (blocking function used) & RTL struct */
static pthread_t     dongle;
static hackrf_device *hackrf_device_ctx = NULL;


/* receiver State & Options */
struct receiver_state {
    /* Variables used for stop conditions */
    volatile bool exit_flag;

    /* Double buffering used for sampling */
    float    iSamples[2][SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE];
    float    qSamples[2][SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE];

    /* Sample index */
    uint32_t iqIndex[2];

    /* Buffer selected (0 or 1) */
    uint32_t bufferIndex;

    /* Time at the beginning of the frame to decode */
    struct tm *gtm;
};

struct receiver_options {
    uint32_t dialfreq;
    uint32_t realfreq;
    int32_t  gain;
    int32_t  autogain;
    int32_t  ppm;
    int32_t  shift;
    int32_t  upconverter;
    int32_t  directsampling;
    int32_t  maxloop;
    int32_t  nloop;
    int32_t  device;
    int32_t  rf_amp;
    int32_t  if_gain;
    int32_t  bb_gain;
    bool     directsampling_user_set;
    bool     noreport;
    bool     selftest;
    bool     writefile;
    bool     readfile;
    char     *filename;
};


/* states & options are shared with other external objects */
struct receiver_state   rx_state;
struct receiver_options rx_options;
struct decoder_options  dec_options;
struct decoder_results  dec_results[50];


/* Could be nice to update this one with the CI */
const char rtlsdr_wsprd_version[] = "0.5.6-hackrf";
const char wsprnet_app_version[]  = "hackrf056";  // 10 chars max.!


/* Callback for each buffer received */
static int hackrf_callback(hackrf_transfer *transfer) {
    int8_t *sigIn = (int8_t *)transfer->buffer;
    uint32_t samples_count = transfer->valid_length;

    /* CIC buffers/vars */
    static int32_t  Ix1 = 0, Ix2 = 0,
                    Qx1 = 0, Qx2 = 0;
    static int32_t  Iy1 = 0, It1y = 0, It1z = 0,
                    Qy1 = 0, Qt1y = 0, Qt1z = 0;
    static int32_t  Iy2 = 0, It2y = 0, It2z = 0,
                    Qy2 = 0, Qt2y = 0, Qt2z = 0;
    static uint32_t decimationIndex = 0;

    /* FIR compensation filter coefs
       Using : Octave/MATLAB code for generating compensation FIR coefficients
       URL : https://github.com/WestCoastDSP/CIC_Octave_Matlab
     */
    static const float zCoef[33] = {
        -0.0027772683, -0.0005058826,  0.0049745750, -0.0034059318,
        -0.0077557814,  0.0139375423,  0.0039896935, -0.0299394142,
         0.0162250643,  0.0405130860, -0.0580746013, -0.0272104968,
         0.1183705475, -0.0306029022, -0.2011241667,  0.1615898423,
         0.5000000000,
         0.1615898423, -0.2011241667, -0.0306029022,  0.1183705475,
        -0.0272104968, -0.0580746013,  0.0405130860,  0.0162250643,
        -0.0299394142,  0.0039896935,  0.0139375423, -0.0077557814,
        -0.0034059318,  0.0049745750, -0.0005058826, -0.0027772683
    };

    /* FIR compensation filter buffers */
    static float firI[FIR_TAPS] = {0.0},
                 firQ[FIR_TAPS] = {0.0};

    /* Economic mixer @ fs/4 (upper band)
       At fs/4, sin and cosin calculations are no longer required.

               0   | pi/2 |  pi  | 3pi/2
             ----------------------------
       sin =   0   |  1   |  0   |  -1  |
       cos =   1   |  0   | -1   |   0  |

       out_I = in_I * cos(x) - in_Q * sin(x)
       out_Q = in_Q * cos(x) + in_I * sin(x)
       (Keep the upper band, IQ inverted on RTL devices)
    */
    int8_t tmp;
    for (uint32_t i = 0; i + 7 < samples_count; i += 8) {
        tmp         =  sigIn[i+3];
        sigIn[i+3]  =  sigIn[i+2];
        sigIn[i+2]  = -tmp;
        sigIn[i+4]  = -sigIn[i+4];
        sigIn[i+5]  = -sigIn[i+5];
        tmp         =  sigIn[i+6];
        sigIn[i+6]  =  sigIn[i+7];
        sigIn[i+7]  = -tmp;
    }

    /* CIC decimator (N=2)
       Info: * Understanding CIC Compensation Filters
               https://www.altera.com/en_US/pdfs/literature/an/an455.pdf
             * Understanding cascaded integrator-comb filters
               https://www.embedded.com/design/configurable-systems/4006446/Understanding-cascaded-integrator-comb-filters
    */
    for (uint32_t i = 0; i < samples_count / 2; i++) {
        /* Integrator stages (N=2) */
        Ix1 += (int32_t)sigIn[i * 2];
        Qx1 += (int32_t)sigIn[i * 2 + 1];
        Ix2 += Ix1;
        Qx2 += Qx1;

        /* Decimation stage */
        decimationIndex++;
        if (decimationIndex <= DOWNSAMPLING) {
            continue;
        }
        decimationIndex = 0;

        /* 1st Comb */
        Iy1  = Ix2 - It1z;
        It1z = It1y;
        It1y = Ix2;
        Qy1  = Qx2 - Qt1z;
        Qt1z = Qt1y;
        Qt1y = Qx2;

        /* 2nd Comd */
        Iy2  = Iy1 - It2z;
        It2z = It2y;
        It2y = Iy1;
        Qy2  = Qy1 - Qt2z;
        Qt2z = Qt2y;
        Qt2y = Qy1;

        /* FIR compensation filter */
        float Isum = 0.0,
              Qsum = 0.0;
        for (uint32_t j = 0; j < FIR_TAPS; j++) {
            Isum += firI[j] * zCoef[j];
            Qsum += firQ[j] * zCoef[j];
            if (j < FIR_TAPS-1) {
                firI[j] = firI[j + 1];
                firQ[j] = firQ[j + 1];
            }
        }
        firI[FIR_TAPS-1] = (float)Iy2;
        firQ[FIR_TAPS-1] = (float)Qy2;
        Isum += firI[FIR_TAPS-1] * zCoef[FIR_TAPS];
        Qsum += firQ[FIR_TAPS-1] * zCoef[FIR_TAPS];

        /* Save the result in the buffer */
        uint32_t idx = rx_state.bufferIndex;
        if (rx_state.iqIndex[idx] < (SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE)) {
            rx_state.iSamples[idx][rx_state.iqIndex[idx]] = Isum;
            rx_state.qSamples[idx][rx_state.iqIndex[idx]] = Qsum;
            rx_state.iqIndex[idx]++;
        }
    }
    return 0;
}


static void sigint_callback_handler(int signum) {
    fprintf(stderr, "Signal caught %d, exiting!\n", signum);
    rx_state.exit_flag = true;
    if (hackrf_device_ctx != NULL) {
        hackrf_stop_rx(hackrf_device_ctx);
    }
}


/* Thread used for this RX blocking function */
static void *hackrf_rx(void *arg) {
    int result = hackrf_start_rx(hackrf_device_ctx, hackrf_callback, NULL);
    if (result != HACKRF_SUCCESS) {
        fprintf(stderr, "ERROR: Failed to start HackRF RX (%s).\n", hackrf_error_name(result));
        rx_state.exit_flag = true;
        return NULL;
    }

    while (!rx_state.exit_flag && hackrf_is_streaming(hackrf_device_ctx) == HACKRF_TRUE) {
        usleep(100000);
    }

    hackrf_stop_rx(hackrf_device_ctx);
    return NULL;
}


/* Thread used for the decoder */
static void *decoder(void *arg) {
    int32_t n_results = 0;

    while (!rx_state.exit_flag) {
        safe_cond_wait(&decState.ready_cond, &decState.ready_mutex);

        LOG(LOG_DEBUG, "Decoder thread -- Got a signal!\n");

        if (rx_state.exit_flag)
            break;  /* Abort case, final sig */

        /* Select the previous transmission / other buffer */
        uint32_t prevBuffer = (rx_state.bufferIndex + 1) % 2;

        if (rx_state.iqIndex[prevBuffer] < ( (SIGNAL_LENGHT - 3) * SIGNAL_SAMPLE_RATE ) ) {
            LOG(LOG_DEBUG, "Decoder thread -- Signal too short, skipping!\n");
            continue;  /* Partial buffer during the first RX, skip it! */
        } else {
            rx_options.nloop++; /* Decoding this signal, count it! */
        }

        /* Delete any previous samples tail */
        for (int i = rx_state.iqIndex[prevBuffer]; i < SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE; i++) {
            rx_state.iSamples[prevBuffer][i] = 0.0;
            rx_state.qSamples[prevBuffer][i] = 0.0;
        }

        /* Normalize the sample @-3dB */
        float maxSig = 1e-24f;
        for (int i = 0; i < SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE; i++) {
            float absI = fabs(rx_state.iSamples[prevBuffer][i]);
            float absQ = fabs(rx_state.qSamples[prevBuffer][i]);

            if (absI > maxSig)
                maxSig = absI;
            if (absQ > maxSig)
                maxSig = absQ;
        }
        maxSig = 0.5 / maxSig;
        for (int i = 0; i < SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE; i++) {
            rx_state.iSamples[prevBuffer][i] *= maxSig;
            rx_state.qSamples[prevBuffer][i] *= maxSig;
        }

        /* Get the date at the beginning last recording session
           with 1 second margin added, just to be sure to be on this even minute
        */
        time_t unixtime;
        time ( &unixtime );
        unixtime = unixtime - 120 + 1;
        rx_state.gtm = gmtime( &unixtime );

        /* Search & decode the signal */
        wspr_decode(rx_state.iSamples[prevBuffer],
                    rx_state.qSamples[prevBuffer],
                    SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE,
                    dec_options,
                    dec_results,
                    &n_results);
        LOG(LOG_DEBUG, "Decoder thread -- Decoding completed\n");
        saveSample(rx_state.iSamples[prevBuffer], rx_state.qSamples[prevBuffer]);
        postSpots(n_results);
        printSpots(n_results);
    }
    return NULL;
}


/* Double buffer management */
void initSampleStorage() {
    rx_state.bufferIndex = 0;
    rx_state.iqIndex[0]  = 0;
    rx_state.iqIndex[1]  = 0;
    rx_state.exit_flag   = false;
}


/* Default options for the receiver */
void initrx_options() {
    rx_options.gain           = 290;
    rx_options.autogain       = 0;
    rx_options.ppm            = 0;
    rx_options.shift          = 0;
    rx_options.directsampling = 0;
    rx_options.maxloop        = 0;
    rx_options.nloop          = 0;
    rx_options.device         = 0;
    rx_options.rf_amp         = -1;
    rx_options.if_gain        = -1;
    rx_options.bb_gain        = -1;
    rx_options.directsampling_user_set = false;
    rx_options.selftest       = false;
    rx_options.writefile      = false;
    rx_options.readfile       = false;
    rx_options.noreport       = false;
}

/* Default options for the decoder */
void initDecoder_options() {
    dec_options.usehashtable  = 0;
    dec_options.npasses       = 2;
    dec_options.subtraction   = 1;
    dec_options.quickmode     = 0;
}


/* Report on WSPRnet */
void postSpots(uint32_t n_results) {
    CURL *curl;
    CURLcode res;
    char url[512];

    if (rx_options.noreport) {
        LOG(LOG_DEBUG, "Decoder thread -- Skipping the reporting\n");
        return;
    }

    curl = curl_easy_init();
    if (!curl)
        return;

    char *esc_rcall = curl_easy_escape(curl, dec_options.rcall, 0);
    char *esc_rloc  = curl_easy_escape(curl, dec_options.rloc, 0);
    if (!esc_rcall || !esc_rloc) {
        curl_free(esc_rcall);
        curl_free(esc_rloc);
        curl_easy_cleanup(curl);
        return;
    }

    if (n_results == 0) {
        snprintf(url, sizeof(url), "https://wsprnet.org/post?function=wsprstat&rcall=%s&rgrid=%s&rqrg=%.6f&tpct=%.2f&tqrg=%.6f&dbm=%d&version=%s&mode=2",
                 esc_rcall,
                 esc_rloc,
                 rx_options.dialfreq / 1e6,
                 0.0f,
                 rx_options.dialfreq / 1e6,
                 0,
                 wsprnet_app_version);

        LOG(LOG_DEBUG, "Decoder thread -- Sending empty report using this URL: %s\n", url);
        curl_easy_setopt(curl, CURLOPT_URL, url);
        curl_easy_setopt(curl, CURLOPT_NOBODY, 1);
        res = curl_easy_perform(curl);

        if (res != CURLE_OK)
            fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));

        curl_free(esc_rcall);
        curl_free(esc_rloc);
        curl_easy_cleanup(curl);
        return;
    }

    for (uint32_t i = 0; i < n_results; i++) {
        snprintf(url, sizeof(url), "https://wsprnet.org/post?function=wspr&rcall=%s&rgrid=%s&rqrg=%.6f&date=%02d%02d%02d&time=%02d%02d&sig=%.0f&dt=%.1f&tqrg=%.6f&tcall=%s&tgrid=%s&dbm=%s&version=%s&mode=2",
                 esc_rcall,
                 esc_rloc,
                 dec_results[i].freq,
                 rx_state.gtm->tm_year - 100,
                 rx_state.gtm->tm_mon + 1,
                 rx_state.gtm->tm_mday,
                 rx_state.gtm->tm_hour,
                 rx_state.gtm->tm_min,
                 dec_results[i].snr,
                 dec_results[i].dt,
                 dec_results[i].freq,
                 dec_results[i].call,
                 dec_results[i].loc,
                 dec_results[i].pwr,
                 wsprnet_app_version);

        LOG(LOG_DEBUG, "Decoder thread -- Sending spot using this URL: %s\n", url);
        curl_easy_reset(curl);
        curl_easy_setopt(curl, CURLOPT_URL, url);
        curl_easy_setopt(curl, CURLOPT_NOBODY, 1);
        res = curl_easy_perform(curl);

        if (res != CURLE_OK)
            fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
    }

    curl_free(esc_rcall);
    curl_free(esc_rloc);
    curl_easy_cleanup(curl);
}


void printSpots(uint32_t n_results) {
    if (n_results == 0) {
        printf("No spot %04d-%02d-%02d %02d:%02dz\n",
               rx_state.gtm->tm_year + 1900,
               rx_state.gtm->tm_mon + 1,
               rx_state.gtm->tm_mday,
               rx_state.gtm->tm_hour,
               rx_state.gtm->tm_min);

        return;
    }

    for (uint32_t i = 0; i < n_results; i++) {
        printf("Spot :  %04d-%02d-%02d %02d:%02dz %6.2f %6.2f %10.6f %2d %7s %6s %2s\n",
               rx_state.gtm->tm_year + 1900,
               rx_state.gtm->tm_mon + 1,
               rx_state.gtm->tm_mday,
               rx_state.gtm->tm_hour,
               rx_state.gtm->tm_min,
               dec_results[i].snr,
               dec_results[i].dt,
               dec_results[i].freq,
               (int)dec_results[i].drift,
               dec_results[i].call,
               dec_results[i].loc,
               dec_results[i].pwr);
    }
}


void saveSample(float *iSamples, float *qSamples) {
    if (rx_options.writefile == true) {
        char filename[64];

        time_t rawtime;
        time(&rawtime);
        struct tm *gtm = gmtime(&rawtime);

        snprintf(filename, sizeof(filename) - 1, "%.8s_%04d-%02d-%02d_%02d-%02d-%02d.iq",
                 rx_options.filename,
                 gtm->tm_year + 1900,
                 gtm->tm_mon + 1,
                 gtm->tm_mday,
                 gtm->tm_hour,
                 gtm->tm_min,
                 gtm->tm_sec);

        writeRawIQfile(iSamples, qSamples, filename);
    }
}


double atofs(char *s) {
    /* standard suffixes */
    char last;
    uint32_t len;
    double suff = 1.0;
    len = strlen(s);
    last = s[len - 1];
    s[len - 1] = '\0';

    switch (last) {
        case 'g':
        case 'G':
            suff *= 1e3;
        case 'm':
        case 'M':
            suff *= 1e3;
        case 'k':
        case 'K':
            suff *= 1e3;
            suff *= atof(s);
            s[len - 1] = last;
            return suff;
    }
    s[len - 1] = last;
    return atof(s);
}


int32_t parse_u64(char *s, uint64_t *const value) {
    uint_fast8_t base = 10;
    char *s_end;
    uint64_t u64_value;

    if (strlen(s) > 2) {
        if (s[0] == '0') {
            if ((s[1] == 'x') || (s[1] == 'X')) {
                base = 16;
                s += 2;
            } else if ((s[1] == 'b') || (s[1] == 'B')) {
                base = 2;
                s += 2;
            }
        }
    }

    s_end = s;
    u64_value = strtoull(s, &s_end, base);
    if ((s != s_end) && (*s_end == 0)) {
        *value = u64_value;
        return 1;
    } else {
        return 0;
    }
}


int32_t readRawIQfile(float *iSamples, float *qSamples, const char *filename) {
    float filebuffer[2 * SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE];
    FILE *fd = fopen(filename, "rb");

    if (fd == NULL) {
        fprintf(stderr, "Cannot open data file...\n");
        return 0;
    }

    /* Read the IQ file */
    int32_t nread = fread(filebuffer, sizeof(float), 2 * SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE, fd);
    int32_t recsize = nread / 2;

    /* Convert the interleaved buffer into 2 buffers */
    for (int32_t i = 0; i < recsize; i++) {
        iSamples[i] =  filebuffer[2 * i];
        qSamples[i] = -filebuffer[2 * i + 1];  // neg, convention used by wsprsim
    }

    /* Normalize the sample @-3dB */
    float maxSig = 1e-24f;
    for (int i = 0; i < recsize; i++) {
        float absI = fabs(iSamples[i]);
        float absQ = fabs(qSamples[i]);

        if (absI > maxSig)
            maxSig = absI;
        if (absQ > maxSig)
            maxSig = absQ;
    }
    maxSig = 0.5 / maxSig;
    for (int i = 0; i <recsize; i++) {
        iSamples[i] *= maxSig;
        qSamples[i] *= maxSig;
    }

    return recsize;
}


int32_t writeRawIQfile(float *iSamples, float *qSamples, const char *filename) {
    float filebuffer[2 * SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE];

    FILE *fd = fopen(filename, "wb");
    if (fd == NULL) {
        fprintf(stderr, "Cannot open data file...\n");
        return 0;
    }

    for (int32_t i = 0; i < SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE; i++) {
        filebuffer[2 * i]     =  iSamples[i];
        filebuffer[2 * i + 1] = -qSamples[i];  // neg, convention used by wsprsim
    }

    int32_t nwrite = fwrite(filebuffer, sizeof(float), 2 * SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE, fd);
    if (nwrite != 2 * SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE) {
        fprintf(stderr, "Cannot write all the data!\n");
        return 0;
    }

    fclose(fd);
    return SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE;
}


int32_t readC2file(float *iSamples, float *qSamples, const char *filename) {
    float filebuffer[2 * SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE];
    FILE *fd = fopen(filename, "rb");
    int32_t nread;
    double frequency;
    int    type;
    char   name[15];

    if (fd == NULL) {
        fprintf(stderr, "Cannot open data file...\n");
        return 0;
    }

    /* Read the header */
    nread = fread(name, sizeof(char), 14, fd);
    nread = fread(&type, sizeof(int), 1, fd);
    nread = fread(&frequency, sizeof(double), 1, fd);
    rx_options.dialfreq = frequency;

    /* Read the IQ file */
    nread = fread(filebuffer, sizeof(float), 2 * SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE, fd);
    int32_t recsize = nread / 2;

    /* Convert the interleaved buffer into 2 buffers */
    for (int32_t i = 0; i < recsize; i++) {
        iSamples[i] =  filebuffer[2 * i];
        qSamples[i] = -filebuffer[2 * i + 1];  // neg, convention used by wsprsim
    }

    /* Normalize the sample @-3dB */
    float maxSig = 1e-24f;
    for (int i = 0; i < recsize; i++) {
        float absI = fabs(iSamples[i]);
        float absQ = fabs(qSamples[i]);

        if (absI > maxSig)
            maxSig = absI;
        if (absQ > maxSig)
            maxSig = absQ;
    }
    maxSig = 0.5 / maxSig;
    for (int i = 0; i <recsize; i++) {
        iSamples[i] *= maxSig;
        qSamples[i] *= maxSig;
    }

    return recsize;
}


void decodeRecordedFile(const char *filename) {
    static float iSamples[SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE] = {0};
    static float qSamples[SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE] = {0};
    static uint32_t samples_len;
    int32_t n_results = 0;

    if (strcmp(&filename[strlen(filename)-3], ".iq") == 0) {
        samples_len = readRawIQfile(iSamples, qSamples, filename);
    } else if (strcmp(&filename[strlen(filename)-3], ".c2") == 0) {
        samples_len = readC2file(iSamples, qSamples, filename);
    } else {
        fprintf(stderr, "Not a valid extension!! (only .iq & .c2 files)\n");
        return;
    }

    printf("Number of samples: %d\n", samples_len);

    if (samples_len) {
        /* Search & decode the signal */
        wspr_decode(iSamples, qSamples, samples_len, dec_options, dec_results, &n_results);

        printf("        SNR      DT        Freq Dr    Call    Loc Pwr\n");
        for (uint32_t i = 0; i < n_results; i++) {
            printf("Spot : %6.2f %6.2f %10.6f %2d %7s %6s %2s\n",
                   dec_results[i].snr,
                   dec_results[i].dt,
                   dec_results[i].freq,
                   (int)dec_results[i].drift,
                   dec_results[i].call,
                   dec_results[i].loc,
                   dec_results[i].pwr);
        }
    }
}


float whiteGaussianNoise(float factor) {
    static double V1, V2, U1, U2, S, X;
    static int phase = 0;

    if (phase == 0) {
        do {
            U1 = rand() / (double)RAND_MAX;
            U2 = rand() / (double)RAND_MAX;
            V1 = 2 * U1 - 1;
            V2 = 2 * U2 - 1;
            S = V1 * V1 + V2 * V2;
        } while(S >= 1 || S == 0);

        X = V1 * sqrt(-2 * log(S) / S);
    } else {
        X = V2 * sqrt(-2 * log(S) / S);
    }

    phase = 1 - phase;
    return (float)X * factor;
}


int32_t decoderSelfTest() {
    static float iSamples[SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE] = {0};
    static float qSamples[SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE] = {0};
    static uint32_t samples_len = SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE;
    int32_t n_results = 0;

    unsigned char symbols[162];
    char message[] = "K1JT FN20QI 20";
    char hashtab[HASHTAB_SIZE * HASHTAB_ENTRY_LEN] = {0};
    char loctab[HASHTAB_SIZE * LOCTAB_ENTRY_LEN]  = {0};

    // Compute sympbols from the message
    get_wspr_channel_symbols(message, hashtab, loctab, symbols);

    float  f0  = 50.0;
    float  t0  = 2.0;  // Caution!! Possible buffer overflow with the index calculation (no user input here!)
    float  amp = 1.0;
    float  wgn = 0.02;
    double phi = 0.0;
    double df  = 375.0 / 256.0;
    double dt  = 1 / 375.0;

    // Add signal
    for (int i = 0; i < 162; i++) {
        double dphi = 2.0 * M_PI * dt * (f0 + ( (double)symbols[i]-1.5) * df);
        for (int j = 0; j < 256; j++) {
            int index = t0 / dt + 256 * i + j;
            iSamples[index] = amp * cos(phi) + whiteGaussianNoise(wgn);
            qSamples[index] = amp * sin(phi) + whiteGaussianNoise(wgn);
            phi += dphi;
        }
    }

    /* Save the test sample */
    writeRawIQfile(iSamples, qSamples, "selftest.iq");

    /* Search & decode the signal */
    wspr_decode(iSamples, qSamples, samples_len, dec_options, dec_results, &n_results);

    printf("        SNR      DT        Freq Dr    Call    Loc Pwr\n");
    for (uint32_t i = 0; i < n_results; i++) {
        printf("Spot(%i) %6.2f %6.2f %10.6f %2d %7s %6s %2s\n",
               i,
               dec_results[i].snr,
               dec_results[i].dt,
               dec_results[i].freq,
               (int)dec_results[i].drift,
               dec_results[i].call,
               dec_results[i].loc,
               dec_results[i].pwr);
    }

    /* Simple consistency check */
    if (strcmp(dec_results[0].call, "K1JT") ||
        strcmp(dec_results[0].loc,  "FN20") ||
        strcmp(dec_results[0].pwr,  "20")) {
        return 0;
    } else {
        return 1;
    }
}


void usage(FILE *stream, int32_t status) {
    fprintf(stream,
            "rtlsdr_wsprd, a simple WSPR daemon for HackRF receivers\n\n"
            "Use:\trtlsdr_wsprd -f frequency -c callsign -l locator [options]\n"
            "\t-f dial frequency [(,k,M) Hz] or band string\n"
            "\t   If band string is used, the default dial frequency will used.\n"
            "\t   Bands: LF MF 160m 80m 60m 40m 30m 20m 17m 15m 12m 10m 6m 4m 2m 1m25 70cm 23cm\n"
            "\t-c your callsign (12 chars max)\n"
            "\t-l your locator grid (6 chars max)\n"
            "Receiver extra options:\n"
            "\t-g gain [0-62] (default: 29, mapped to HackRF VGA/LNA)\n"
            "\t-A RF amp [0,1] (HackRF front-end amp, optional)\n"
            "\t-L IF gain [0-40] step 8 (HackRF LNA gain, optional)\n"
            "\t-V baseband gain [0-62] step 2 (HackRF VGA gain, optional)\n"
            "\t-a auto gain (accepted but ignored on HackRF)\n"
            "\t-o frequency offset (default: 0)\n"
            "\t-p crystal correction factor (ppm) (accepted but ignored on HackRF)\n"
            "\t-u upconverter (default: 0, example: 125M)\n"
            "\t-d direct sampling [0,1,2] (accepted but ignored on HackRF)\n"
            "\t-n max iterations (default: 0 = infinite loop)\n"
            "\t-i HackRF device index (in case of multiple receivers, default: 0)\n"
            "Decoder extra options:\n"
            "\t-H use the hash table (could caught signal 11 on RPi, no parameter)\n"
            "\t-Q quick mode, doesn't dig deep for weak signals, no parameter\n"
            "\t-S single pass mode, no subtraction (same as original wsprd), no parameter\n"
            "\t-x do not report any spots on web clusters (WSPRnet, PSKreporter...)\n"
            "Debugging options:\n"
            "\t-t decoder self-test (generate a signal & decode), no parameter\n"
            "\t-w write received signal and exit [filename prefix]\n"
            "\t-r read signal with .iq or .c2 format, decode and exit [filename]\n"
            "\t   (raw format: 375sps, float 32 bits, 2 channels)\n"
            "Other options:\n"
            "\t--help show list of options\n"
            "\t--version show version of program\n"
            "Example:\n"
            "\trtlsdr_wsprd -f 2m -c A1XYZ -l AB12cd -g 29 -o -4200\n");
    exit(status);
}


int main(int argc, char **argv) {
    uint32_t opt;
    char    *short_options = "f:c:l:g:A:L:V:ao:p:u:d:n:i:tw:r:HQSx";
    int32_t  option_index = 0;
    struct option long_options[] = {
        {"help",    no_argument, 0, 0 },
        {"version", no_argument, 0, 0 },
        {0, 0, 0, 0 }
    };

    int32_t hackrf_result;
    hackrf_device_list_t *hackrf_list = NULL;

    initrx_options();
    initDecoder_options();

    if (argc <= 1)
        usage(stdout, EXIT_SUCCESS);

    while ((opt = getopt_long(argc, argv, short_options, long_options, &option_index)) != -1) {
        switch (opt) {
            case 0:
                switch (option_index) {
                    case 0:  // --help
                        usage(stdout, EXIT_SUCCESS);
                        break;
                    case 1:  // --version
                        printf("rtlsdr_wsprd v%s\n", rtlsdr_wsprd_version);
                        exit(EXIT_SUCCESS);
                        break;
                }
                break;
            case 'f':  // Frequency
                if (!strcasecmp(optarg, "LF")) {
                    rx_options.dialfreq = 136000;
                    // Implicit direct sampling for HF bands & lower
                    if (!rx_options.directsampling)
                        rx_options.directsampling = 2;
                } else if (!strcasecmp(optarg, "MF")) {
                    rx_options.dialfreq = 474200;
                    if (!rx_options.directsampling)
                        rx_options.directsampling = 2;
                } else if (!strcasecmp(optarg, "160m")) {
                    rx_options.dialfreq = 1836600;
                    if (!rx_options.directsampling)
                        rx_options.directsampling = 2;
                } else if (!strcasecmp(optarg, "80m")) {
                    rx_options.dialfreq = 3568600;
                    if (!rx_options.directsampling)
                        rx_options.directsampling = 2;
                } else if (!strcasecmp(optarg, "60m")) {
                    rx_options.dialfreq = 5287200;
                    if (!rx_options.directsampling)
                        rx_options.directsampling = 2;
                } else if (!strcasecmp(optarg, "40m")) {
                    rx_options.dialfreq = 7038600;
                    if (!rx_options.directsampling)
                        rx_options.directsampling = 2;
                } else if (!strcasecmp(optarg, "30m")) {
                    rx_options.dialfreq = 10138700;
                    if (!rx_options.directsampling)
                        rx_options.directsampling = 2;
                } else if (!strcasecmp(optarg, "20m")) {
                    rx_options.dialfreq = 14095600;
                    if (!rx_options.directsampling)
                        rx_options.directsampling = 2;
                } else if (!strcasecmp(optarg, "17m")) {
                    rx_options.dialfreq = 18104600;
                    if (!rx_options.directsampling)
                        rx_options.directsampling = 2;
                } else if (!strcasecmp(optarg, "15m")) {
                    rx_options.dialfreq = 21094600;
                    if (!rx_options.directsampling)
                        rx_options.directsampling = 2;
                } else if (!strcasecmp(optarg, "12m")) {
                    rx_options.dialfreq = 24924600;
                    if (!rx_options.directsampling)
                        rx_options.directsampling = 2;
                } else if (!strcasecmp(optarg, "10m")) {
                    rx_options.dialfreq = 28124600;
                    if (!rx_options.directsampling)
                        rx_options.directsampling = 2;
                } else if (!strcasecmp(optarg, "6m")) {
                    rx_options.dialfreq = 50293000;
                } else if (!strcasecmp(optarg, "4m")) {
                    rx_options.dialfreq = 70091000;
                } else if (!strcasecmp(optarg, "2m")) {
                    rx_options.dialfreq = 144489000;
                } else if (!strcasecmp(optarg, "1m25")) {
                    rx_options.dialfreq = 222280000;
                } else if (!strcasecmp(optarg, "70cm")) {
                    rx_options.dialfreq = 432300000;
                } else if (!strcasecmp(optarg, "23cm")) {
                    rx_options.dialfreq = 1296500000;
                } else {
                    rx_options.dialfreq = (uint32_t)atofs(optarg);
                }
                break;
            case 'c':  // Callsign
                snprintf(dec_options.rcall, sizeof(dec_options.rcall), "%.12s", optarg);
                break;
            case 'l':  // Locator / Grid
                snprintf(dec_options.rloc, sizeof(dec_options.rloc), "%.6s", optarg);
                break;
            case 'g':  // Small signal amplifier gain
                rx_options.gain = atoi(optarg);
                if (rx_options.gain < 0) rx_options.gain = 0;
                if (rx_options.gain > 62) rx_options.gain = 62;
                rx_options.gain *= 10;
                break;
            case 'A':  // HackRF RF amp enable
                rx_options.rf_amp = atoi(optarg) ? 1 : 0;
                break;
            case 'L':  // HackRF IF gain (LNA)
                rx_options.if_gain = atoi(optarg);
                if (rx_options.if_gain < 0) rx_options.if_gain = 0;
                if (rx_options.if_gain > 40) rx_options.if_gain = 40;
                rx_options.if_gain = (rx_options.if_gain / 8) * 8;
                break;
            case 'V':  // HackRF baseband gain (VGA)
                rx_options.bb_gain = atoi(optarg);
                if (rx_options.bb_gain < 0) rx_options.bb_gain = 0;
                if (rx_options.bb_gain > 62) rx_options.bb_gain = 62;
                if (rx_options.bb_gain % 2 != 0) rx_options.bb_gain--;
                break;
            case 'a':  // Auto gain
                rx_options.autogain = 1;
                break;
            case 'o':  // Fine frequency correction
                rx_options.shift = atoi(optarg);
                break;
            case 'p':  // Crystal correction
                rx_options.ppm = atoi(optarg);
                break;
            case 'u':  // Upconverter frequency
                rx_options.upconverter = (uint32_t)atofs(optarg);
                break;
            case 'd':  // Direct Sampling
                rx_options.directsampling = (uint32_t)atofs(optarg);
                rx_options.directsampling_user_set = true;
                break;
            case 'n':  // Stop after n iterations
                rx_options.maxloop = (uint32_t)atofs(optarg);
                break;
            case 'i':  // Select the device to use
                rx_options.device = (uint32_t)atofs(optarg);
                break;
            case 'H':  // Decoder option, use a hashtable
                dec_options.usehashtable = 1;
                break;
            case 'Q':  // Decoder option, faster
                dec_options.quickmode = 1;
                break;
            case 'S':  // Decoder option, single pass mode (same as original wsprd)
                dec_options.subtraction = 0;
                dec_options.npasses = 1;
                break;
            case 'x':  // Decoder option, single pass mode (same as original wsprd)
                rx_options.noreport = true;
                break;
            case 't':  // Seft test (used in unit-test CI pipeline)
                rx_options.selftest = true;
                break;
            case 'w':  // Write a signal and exit
                rx_options.writefile = true;
                rx_options.filename = optarg;
                break;
            case 'r':  // Read a signal and decode
                rx_options.readfile = true;
                rx_options.filename = optarg;
                break;
            default:
                usage(stderr, EXIT_FAILURE);
                break;
        }
    }

    if (rx_options.dialfreq == 0) {
        fprintf(stderr, "Please specify a dial frequency.\n");
        fprintf(stderr, " --help for usage...\n");
        return EXIT_FAILURE;
    }

    if (dec_options.rcall[0] == 0) {
        fprintf(stderr, "Please specify your callsign.\n");
        fprintf(stderr, " --help for usage...\n");
        return EXIT_FAILURE;
    }

    if (dec_options.rloc[0] == 0) {
        fprintf(stderr, "Please specify your locator.\n");
        fprintf(stderr, " --help for usage...\n");
        return EXIT_FAILURE;
    }

    /* Calcule shift offset */
    rx_options.realfreq = rx_options.dialfreq + rx_options.shift + rx_options.upconverter;

    /* Store the frequency used for the decoder */
    dec_options.freq = rx_options.dialfreq;

    if (rx_options.selftest == true) {
        if (decoderSelfTest()) {
            fprintf(stdout, "Self-test SUCCESS!\n");
            return EXIT_SUCCESS;
        }
        else {
            fprintf(stderr, "Self-test FAILED!\n");
            return EXIT_FAILURE;
        }
    }

    if (rx_options.readfile == true) {
        fprintf(stdout, "Reading IQ file: %s\n", rx_options.filename);
        decodeRecordedFile(rx_options.filename);
        return EXIT_SUCCESS;
    }

    if (rx_options.writefile == true) {
        fprintf(stdout, "Saving IQ file planned with prefix: %.8s\n", rx_options.filename);
    }

    /* If something goes wrong... */
    signal(SIGINT,  &sigint_callback_handler);
    signal(SIGTERM, &sigint_callback_handler);
    signal(SIGILL,  &sigint_callback_handler);
    signal(SIGFPE,  &sigint_callback_handler);
    signal(SIGSEGV, &sigint_callback_handler);
    signal(SIGABRT, &sigint_callback_handler);

    /* Init & parameter the device */
    hackrf_result = hackrf_init();
    if (hackrf_result != HACKRF_SUCCESS) {
        fprintf(stderr, "ERROR: Failed to initialize HackRF library (%s).\n", hackrf_error_name(hackrf_result));
        return EXIT_FAILURE;
    }

    hackrf_list = hackrf_device_list();
    if (!hackrf_list || hackrf_list->devicecount <= 0) {
        fprintf(stderr, "No supported HackRF devices found\n");
        if (hackrf_list) {
            hackrf_device_list_free(hackrf_list);
        }
        hackrf_exit();
        return EXIT_FAILURE;
    }

    fprintf(stderr, "Found %d HackRF device(s):\n", hackrf_list->devicecount);
    for (int i = 0; i < hackrf_list->devicecount; i++) {
        const char *serial = "unknown";
        if (hackrf_list->serial_numbers && hackrf_list->serial_numbers[i]) {
            serial = hackrf_list->serial_numbers[i];
        }
        fprintf(stderr, "  %d: SN: %s\n", i, serial);
    }
    fprintf(stderr, "\nUsing HackRF device %d\n", rx_options.device);

    hackrf_result = hackrf_device_list_open(hackrf_list, rx_options.device, &hackrf_device_ctx);
    hackrf_device_list_free(hackrf_list);
    hackrf_list = NULL;
    if (hackrf_result != HACKRF_SUCCESS) {
        fprintf(stderr, "ERROR: Failed to open HackRF device #%d (%s).\n", rx_options.device, hackrf_error_name(hackrf_result));
        hackrf_exit();
        return EXIT_FAILURE;
    }

    if (rx_options.directsampling && rx_options.directsampling_user_set) {
        fprintf(stderr, "WARN: Direct sampling is not supported on HackRF. Ignoring -d option.\n");
    }

    hackrf_result = hackrf_set_sample_rate_manual(hackrf_device_ctx, SAMPLING_RATE, 1);
    if (hackrf_result != HACKRF_SUCCESS) {
        hackrf_result = hackrf_set_sample_rate(hackrf_device_ctx, (double)SAMPLING_RATE);
    }
    if (hackrf_result != HACKRF_SUCCESS) {
        fprintf(stderr, "ERROR: Failed to set sample rate (%s).\n", hackrf_error_name(hackrf_result));
        hackrf_close(hackrf_device_ctx);
        hackrf_exit();
        return EXIT_FAILURE;
    }

    uint32_t filter_bw = hackrf_compute_baseband_filter_bw_round_down_lt(SAMPLING_RATE);
    hackrf_result = hackrf_set_baseband_filter_bandwidth(hackrf_device_ctx, filter_bw);
    if (hackrf_result != HACKRF_SUCCESS) {
        fprintf(stderr, "ERROR: Failed to set baseband filter (%s).\n", hackrf_error_name(hackrf_result));
        hackrf_close(hackrf_device_ctx);
        hackrf_exit();
        return EXIT_FAILURE;
    }

    if (rx_options.autogain) {
        fprintf(stderr, "WARN: Auto gain is not supported on HackRF. Using manual gain mapping.\n");
    }

    if (rx_options.if_gain >= 0 || rx_options.bb_gain >= 0 || rx_options.rf_amp >= 0) {
        uint32_t lna_gain = (rx_options.if_gain >= 0) ? (uint32_t)rx_options.if_gain : 16;
        uint32_t vga_gain = (rx_options.bb_gain >= 0) ? (uint32_t)rx_options.bb_gain : 32;
        uint8_t amp_enable = (rx_options.rf_amp >= 0) ? (uint8_t)rx_options.rf_amp : 0;

        hackrf_result = hackrf_set_lna_gain(hackrf_device_ctx, lna_gain);
        if (hackrf_result != HACKRF_SUCCESS) {
            fprintf(stderr, "ERROR: Failed to set LNA gain (%s).\n", hackrf_error_name(hackrf_result));
            hackrf_close(hackrf_device_ctx);
            hackrf_exit();
            return EXIT_FAILURE;
        }

        hackrf_result = hackrf_set_vga_gain(hackrf_device_ctx, vga_gain);
        if (hackrf_result != HACKRF_SUCCESS) {
            fprintf(stderr, "ERROR: Failed to set VGA gain (%s).\n", hackrf_error_name(hackrf_result));
            hackrf_close(hackrf_device_ctx);
            hackrf_exit();
            return EXIT_FAILURE;
        }

        hackrf_result = hackrf_set_amp_enable(hackrf_device_ctx, amp_enable);
        if (hackrf_result != HACKRF_SUCCESS) {
            fprintf(stderr, "ERROR: Failed to set RF amp (%s).\n", hackrf_error_name(hackrf_result));
            hackrf_close(hackrf_device_ctx);
            hackrf_exit();
            return EXIT_FAILURE;
        }
    } else {
        uint32_t gain_db = (uint32_t)(rx_options.gain / 10);
        uint32_t vga_gain = gain_db > 62 ? 62 : gain_db;
        if (vga_gain % 2 != 0) {
            vga_gain--;
        }
        uint32_t lna_gain = gain_db > 40 ? 40 : gain_db;
        lna_gain = (lna_gain / 8) * 8;

        hackrf_result = hackrf_set_lna_gain(hackrf_device_ctx, lna_gain);
        if (hackrf_result != HACKRF_SUCCESS) {
            fprintf(stderr, "ERROR: Failed to set LNA gain (%s).\n", hackrf_error_name(hackrf_result));
            hackrf_close(hackrf_device_ctx);
            hackrf_exit();
            return EXIT_FAILURE;
        }

        hackrf_result = hackrf_set_vga_gain(hackrf_device_ctx, vga_gain);
        if (hackrf_result != HACKRF_SUCCESS) {
            fprintf(stderr, "ERROR: Failed to set VGA gain (%s).\n", hackrf_error_name(hackrf_result));
            hackrf_close(hackrf_device_ctx);
            hackrf_exit();
            return EXIT_FAILURE;
        }

        hackrf_result = hackrf_set_amp_enable(hackrf_device_ctx, gain_db > 45 ? 1 : 0);
        if (hackrf_result != HACKRF_SUCCESS) {
            fprintf(stderr, "ERROR: Failed to set RF amp (%s).\n", hackrf_error_name(hackrf_result));
            hackrf_close(hackrf_device_ctx);
            hackrf_exit();
            return EXIT_FAILURE;
        }
    }

    if (rx_options.ppm != 0) {
        fprintf(stderr, "WARN: Frequency correction (PPM) is not supported by libhackrf in this daemon.\n");
    }

    hackrf_result = hackrf_set_freq(hackrf_device_ctx, (uint64_t)(rx_options.realfreq + FS4_RATE + 1500));
    if (hackrf_result != HACKRF_SUCCESS) {
        fprintf(stderr, "ERROR: Failed to set frequency (%s).\n", hackrf_error_name(hackrf_result));
        hackrf_close(hackrf_device_ctx);
        hackrf_exit();
        return EXIT_FAILURE;
    }

    /* Date-time info & alignment */
    struct timeval lTime;
    time_t rawtime;
    time ( &rawtime );
    struct tm *gtm = gmtime(&rawtime);

    /* Print used parameter */
    printf("\nStarting hackrf-wsprd (%04d-%02d-%02d, %02d:%02dz) -- Version %s\n",
           gtm->tm_year + 1900, gtm->tm_mon + 1, gtm->tm_mday, gtm->tm_hour, gtm->tm_min, rtlsdr_wsprd_version);
    printf("  Callsign     : %s\n",    dec_options.rcall);
    printf("  Locator      : %s\n",    dec_options.rloc);
    printf("  Dial freq.   : %d Hz\n", rx_options.dialfreq);
    printf("  Real freq.   : %d Hz\n", rx_options.realfreq);
    printf("  PPM factor   : %d\n",    rx_options.ppm);
    if (rx_options.autogain)
        printf("  Auto gain    : enable\n");
    else
        printf("  Gain         : %d dB\n", rx_options.gain / 10);

    /* Wait for timing alignment */
    gettimeofday(&lTime, NULL);
    uint32_t sec   = lTime.tv_sec % 120;
    uint32_t usec  = sec * 1000000 + lTime.tv_usec;
    uint32_t uwait = 120000000 - usec;
    printf("Wait for time sync (start in %d sec)\n\n", uwait / 1000000);
    printf("              Date   Time    SNR     DT       Freq Dr    Call    Loc Pwr\n");

    /* Prepare a low priority param for the decoder thread */
    struct sched_param param;
    pthread_attr_init(&decState.tattr);
    pthread_attr_setschedpolicy(&decState.tattr, SCHED_RR);
    pthread_attr_getschedparam(&decState.tattr, &param);
    param.sched_priority = 90;  // = sched_get_priority_min();
    pthread_attr_setschedparam(&decState.tattr, &param);

    /* Create a thread and stuff for separate decoding
       Info : https://computing.llnl.gov/tutorials/pthreads/
    */
    pthread_cond_init(&decState.ready_cond, NULL);
    pthread_mutex_init(&decState.ready_mutex, NULL);
    pthread_create(&dongle, NULL, hackrf_rx, NULL);
    pthread_create(&decState.thread, &decState.tattr, decoder, NULL);

    /* Main loop : Wait, read, decode */
    while (!rx_state.exit_flag && !(rx_options.maxloop && (rx_options.nloop >= rx_options.maxloop))) {
        /* Wait for time Sync on 2 mins */
        gettimeofday(&lTime, NULL);
        sec   = lTime.tv_sec % 120;
        usec  = sec * 1000000 + lTime.tv_usec;
        uwait = 120000000 - usec;
        LOG(LOG_DEBUG, "Main thread -- Waiting %d seconds\n", uwait/1000000);
        usleep(uwait);
        LOG(LOG_DEBUG, "Main thread -- Sending a GO to the decoder thread\n");

        /* Switch to the other buffer and trigger the decoder */
        rx_state.bufferIndex = (rx_state.bufferIndex + 1) % 2;
        rx_state.iqIndex[rx_state.bufferIndex] = 0;
        safe_cond_signal(&decState.ready_cond, &decState.ready_mutex);
        usleep(100000); /* Give a chance to the other thread to update the nloop counter */
    }

    /* Stop the decoder thread */
    rx_state.exit_flag = true;
    safe_cond_signal(&decState.ready_cond, &decState.ready_mutex);

    /* Stop the RX and free the blocking function */
    if (hackrf_device_ctx != NULL) {
        hackrf_stop_rx(hackrf_device_ctx);
        hackrf_close(hackrf_device_ctx);
        hackrf_device_ctx = NULL;
    }
    hackrf_exit();

    /* Wait the thread join (send a signal before to terminate the job) */
    pthread_join(decState.thread, NULL);
    pthread_join(dongle, NULL);

    /* Destroy the lock/cond/thread */
    pthread_cond_destroy(&decState.ready_cond);
    pthread_mutex_destroy(&decState.ready_mutex);

    printf("Bye!\n");

    return EXIT_SUCCESS;
}
