/**
 * ------------------------------------------------------------------------------*
 * File: panTompkins.c                                                           *
 *       ANSI-C implementation of Pan-Tompkins real-time QRS detection algorithm *
 * Author: Rafael de Moura Moreira <rafaelmmoreira@gmail.com>                    *
 * License: MIT License                                                          *
 * ------------------------------------------------------------------------------*
 * ---------------------------------- HISTORY ---------------------------------- *
 *    date   |    author    |                     description                    *
 * ----------| -------------| ---------------------------------------------------*
 * 2019/04/11| Rafael M. M. | - Fixed moving-window integral.                    *
 *           |              | - Fixed how to find the correct sample with the    *
 *           |              | last QRS.                                          *
 *           |              | - Replaced constant value in code by its #define.  *
 *           |              | - Added some casting on comparisons to get rid of  *
 *           |              | compiler warnings.                                 *
 * 2019/04/15| Rafael M. M. | - Removed delay added to the output by the filters.*
 *           |              | - Fixed multiple detection of the same peak.       *
 * 2019/04/16| Rafael M. M. | - Added output buffer to correctly output a peak   *
 *           |              | found by back searching using the 2nd thresholds.  *
 * 2019/04/23| Rafael M. M. | - Improved comparison of slopes.                   *
 *           |              | - Fixed formula to obtain the correct sample from  *
 *           |              | the buffer on the back search.                     *
 * 2020/07/01| myd          | -                                                  *
 * ------------------------------------------------------------------------------*
 * MIT License                                                                   *
 *                                                                               *
 * Copyright (c) 2018 Rafael de Moura Moreira                                    *
 *                                                                               *
 * Permission is hereby granted, free of charge, to any person obtaining a copy  *
 * of this software and associated documentation files (the "Software"), to deal *
 * in the Software without restriction, including without limitation the rights  *
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell     *
 * copies of the Software, and to permit persons to whom the Software is         *
 * furnished to do so, subject to the following conditions:                      *
 *                                                                               *
 * The above copyright notice and this permission notice shall be included in all*
 * copies or substantial portions of the Software.                               *
 *                                                                               *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,      *
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE   *
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER        *
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, *
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE *
 * SOFTWARE.                                                                     *
 *-------------------------------------------------------------------------------*
 * Description                                                                   *
 *                                                                               *
 * The main goal of this implementation is to be easy to port to different opera-*
 * ting systems, as well as different processors and microcontrollers, including *
 * embedded systems. It can work both online or offline, depending on whether all*
 * the samples are available or not - it can be adjusted on the input function.  *
 *                                                                               *
 * The main function, panTompkings(), calls input() to get the next sample and   *
 * store it in a buffer. Then it runs through a chain of filters: DC block, low  *
 * pass @ 15 Hz and high pass @ 5Hz. The filtered signal goes both through a de- *
 * rivative filter, which output is then squared, and through a windowed-integra-*
 * tor.                                                                          *
 *                                                                               *
 * For a signal peak to be recognized as a fiducial point, its correspondent va- *
 * lue on both the filtered signal and the integrator must be above a certain    *
 * threshold. Additionally, there are time-restraints to prevent a T-wave from   *
 * being mistakenly identified as an R-peak: a hard 200ms restrain (a new peak   *
 * 200ms from the previous one is, necessarily, a T-wave) and a soft 360ms res-  *
 * train (the peak's squared slope must also be very high to be considered as a  *
 * real peak).                                                                   *
 *                                                                               *
 * When a peak candidate is discarded, its value is used to update the noise     *
 * thresholds - which are also used to estimate the peak thresholds.             *
 *                                                                               *
 * Two buffers keep 8 RR-intervals to calculate RR-averages: one of them keeps   *
 * the last 8 RR-intervals, while the other keeps only the RR-intervals that res-*
 * pect certain restrictions. If both averages are equal, the heart pace is con- *
 * sidered normal. If the heart rate isn't normal, the thresholds change to make *
 * it easier to detect possible weaker peaks. If no peak is detected for a long  *
 * period of time, the thresholds also change and the last discarded peak candi- *
 * date is reconsidered.                                                         *
 *-------------------------------------------------------------------------------*
 * Instructions                                                                  *
 *                                                                               *
 * Here's what you should change to adjust the code to your needs:               *
 *                                                                               *
 * On panTompkins.h:                                                             *
 * - typedef int dataType;                                                       *
 * Change it from 'int' to whatever format your data is (float, unsigned int etc)*
 *                                                                               *
 * On both panTompkins.h and panTompkins.c:                                      *
 * - void init(char file_in[], char file_out[]);                                 *
 * This function is meant to do any kind of initial setup, such as starting a    *
 * serial connection with an ECG sensor. Change its parameters to whatever info  *
 * you need and its content. The test version included here loads 2 plain text   *
 * files: an input file, with the signal as a list of integer numbers in ASCII   *
 * format and an output file where either 0 or 1 will be written for each sample,*
 * whether a peak was detected or not.                                           *
 *                                                                               *
 * On panTompkins.c:                                                             *
 * - #define WINDOWSIZE                                                          *
 * Defines the size of the integration window. The original authors suggest on   *
 * their 1985 paper a 150ms window.                                              *
 *                                                                               *
 * - #define FS                                                                  *
 * Defines the sampling frequency.                                               *
 *                                                                               *
 * - #define NOSAMPLE                                                            *
 * A value to indicate you don't have any more samples to read. Choose a value   *
 * which a sample couldn't possibly have (e.g.: a negative value if your A/D con-*
 * verter only works with positive integers).                                    *
 *                                                                               *
 * - #define BUFFSIZE                                                            *
 * The size of the signal buffers. It should fit at least 1.66 times an RR-inter-*
 * val. Heart beats should be between 60 and 80 BPS for humans. So, considering  *
 * 1.66 times 1 second should be safe.                                           *
 *                                                                               *
 * - #define DELAY 22                                                            *
 * The delay introduced to the output signal. The first DELAY samples will be ig-*
 * nored, as the filters add a delay to the output signal, causing a mismatch    *
 * between the input and output signals. It's easier to compare them this way.   *
 * If you need them both to have the same amount of samples, set this to 0. If   *
 * you're working with different filters and/or sampling rates, you might need to*
 * adjust this value.                                                            *
 *                                                                               *
 * - #include <stdio.h>                                                          *
 * The file, as it is, both gets its inputs and sends its outputs to files. It   *
 * works on both Windows and Linux. If your source isn't a file, and/or your sys-*
 * tem doesn't have the <stdio.h> header, remove it.                             *
 * Include any other headers you might need to make your implementation work,    *
 * such as hardware libraries provided by your microcontroller manufacturer.     *
 *                                                                               *
 * - The input() function                                                        *
 * Change it to get the next sample from your source (a file, a serial device etc*
 * previously set up in your init() function. Return the sample value or NOSAMPLE*
 * if there are no more available samples.                                       *
 *                                                                               *
 * - The output() function                                                       *
 * Change it to output whatever you see fit, however you see fit: an RR-interval *
 * (which can be sent as a parameter to your function using the RR arrays), the  *
 * index of sample or timestamp which caused a R peak, whether a sample was a R  *
 * peak or not etc, and it can be written on a file, displayed on screen, blink a*
 * LED etc.                                                                      *
 *                                                                               *
 * - The panTompkins() function                                                  *
 * This function is almost entirely ANSI C, which means it should work as is on  *
 * most platforms. The only lines you really have to change are the fclose() ones*
 * at the very end, which are only here to allow testing of the code on systems  *
 * such as Windows and Linux for PC. You may wish to create extra variables or   *
 * increase the buffers' size as you see fit to extract different kinds of infor-*
 * mation to output, or fine tune the detection as you see fit - such as adding  *
 * different conditions for verification, initializing self-updating variables   *
 * with empirically-obtained values or changing the filters.                     *
 *-------------------------------------------------------------------------------*
 */

#include "panTompkins.h"

#include <assert.h>
#include <stdlib.h>

//#define WINDOWSIZE 20
//#define FS 360
//#define BUFFSIZE 600
//#define DELAY 22

#define BUFFER_LENGTH_IN_SECONDS (10)

struct pan_tompkins_context_tag
{
    // Sampling frequency.
    int fs;
    
    // Delay introduced by the filters. Filter only output samples after this one.
	// Set to 0 if you want to keep the delay. Fixing the delay results in DELAY less samples
	// in the final end result.
    int delay;
    
    // The size of the buffers (in samples). Must fit more than 1.66 times an RR interval, which
    // typically could be around 1 second.
    size_t buffer_size;
    
    // Integrator window size, in samples. The article recommends 150ms. So, FS*0.15.
	// However, you should check empirically if the waveform looks ok.
    size_t window_size;
    
    // The signal array is where the most recent samples are kept. The other arrays are the outputs of each
    // filtering module: DC Block, low pass, high pass, integral etc.
	// The output is a buffer where we can change a previous result (using a back search) before outputting.
	pan_tompkins_scalar_t *signal;
    pan_tompkins_scalar_t *dcblock;
    pan_tompkins_scalar_t *lowpass;
    pan_tompkins_scalar_t *highpass;
    pan_tompkins_scalar_t *derivative;
    pan_tompkins_scalar_t *squared;
    pan_tompkins_scalar_t *integral;
    int *outputSignal;

	// rr1 holds the last 8 RR intervals. rr2 holds the last 8 RR intervals between rrlow and rrhigh.
	// rravg1 is the rr1 average, rr2 is the rravg2. rrlow = 0.92*rravg2, rrhigh = 1.08*rravg2 and rrmiss = 1.16*rravg2.
	// rrlow is the lowest RR-interval considered normal for the current heart beat, while rrhigh is the highest.
	// rrmiss is the longest that it would be expected until a new QRS is detected. If none is detected for such
	// a long interval, the thresholds must be adjusted.
	double rr1[8], rr2[8], rravg1, rravg2, rrlow, rrhigh, rrmiss;
    
    // sample counts how many samples have been read so far.
    size_t sample;

	// lastQRS stores which was the last sample read when the last R sample was
    // triggered.
	size_t lastQRS;
    
    // lastSlope stores the value of the squared slope when the last R sample was triggered.
    pan_tompkins_scalar_t lastSlope;

	// There are the variables from the original Pan-Tompkins algorithm.
	// The ones ending in _i correspond to values from the integrator.
	// The ones ending in _f correspond to values from the DC-block/low-pass/high-pass filtered signal.
	// The peak variables are peak candidates: signal values above the thresholds.
	// The threshold 1 variables are the threshold variables. If a signal sample is higher than this threshold, it's a peak.
	// The threshold 2 variables are half the threshold 1 ones. They're used for a back search when no peak is detected for too long.
	// The spk and npk variables are, respectively, running estimates of signal and noise peaks.
    pan_tompkins_scalar_t spk_i;
    pan_tompkins_scalar_t npk_i;
    pan_tompkins_scalar_t spk_f;
    pan_tompkins_scalar_t npk_f;
    pan_tompkins_scalar_t threshold_i1;
    pan_tompkins_scalar_t threshold_f1;
    pan_tompkins_scalar_t threshold_i2;
    pan_tompkins_scalar_t threshold_f2;

	// regular tells whether the heart pace is regular or not.
	int regular;
};


pan_tompkins_context_t pan_tompkins_init(int fs, int delay, int window_size, int buffer_size)
{
    pan_tompkins_context_t context;

    if (fs <= 100 || delay < 0 || window_size <= 0 || buffer_size <= 0)
        return NULL;
    
    context = calloc(1, sizeof(struct pan_tompkins_context_tag));
    if (context == NULL)
        return NULL;
    
    context->fs = fs;
    context->delay = delay;
    context->window_size = window_size;

    context->buffer_size = buffer_size; // fs * BUFFER_LENGTH_IN_SECONDS;
    
	context->signal = calloc(context->buffer_size, sizeof(pan_tompkins_scalar_t));
    context->dcblock = calloc(context->buffer_size, sizeof(pan_tompkins_scalar_t));
    context->lowpass = calloc(context->buffer_size, sizeof(pan_tompkins_scalar_t));
    context->highpass = calloc(context->buffer_size, sizeof(pan_tompkins_scalar_t));
    context->derivative = calloc(context->buffer_size, sizeof(pan_tompkins_scalar_t));
    context->squared = calloc(context->buffer_size, sizeof(pan_tompkins_scalar_t));
    context->integral = calloc(context->buffer_size, sizeof(pan_tompkins_scalar_t));
    context->outputSignal = calloc(context->buffer_size, sizeof(int));
    if (context->signal == NULL ||
        context->dcblock == NULL ||
        context->lowpass == NULL ||
        context->highpass == NULL ||
        context->derivative == NULL ||
        context->squared == NULL ||
        context->integral == NULL ||
        context->outputSignal == NULL)
    {
        pan_tompkins_free(&context);
        return NULL;
    }
    
    context->regular = 1;

    return context;
}


int pan_tompkins_run(pan_tompkins_context_t context, pan_tompkins_scalar_t x)
{
    int output;
    pan_tompkins_run_many(context, &x, 1, &output);
    return output;
}


/*
    This is the actual QRS-detecting function. It's a loop that constantly calls the input and output functions
    and updates the thresholds and averages until there are no more samples. More details both above and in
    shorter comments below.
*/
void pan_tompkins_run_many(pan_tompkins_context_t context, const pan_tompkins_scalar_t *data, size_t len, int *output)
{
    size_t c;
    size_t i, j;

	// currentSlope helps calculate the max. square slope for the present sample.
	// lastSlope and currentSlope are all long unsigned int so that very long signals can be read without messing the count.
    pan_tompkins_scalar_t currentSlope = 0;

	// This variable is used as an index to work with the signal buffers. If the buffers still aren't
	// completely filled, it shows the last filled position. Once the buffers are full, it'll always
	// show the last position, and new samples will make the buffers shift, discarding the oldest
	// sample and storing the newest one on the last position.
	size_t current;

	// There are the variables from the original Pan-Tompkins algorithm.
	// The ones ending in _i correspond to values from the integrator.
	// The ones ending in _f correspond to values from the DC-block/low-pass/high-pass filtered signal.
	pan_tompkins_scalar_t peak_i = 0, peak_f = 0;

	// qrs tells whether there was a detection or not.
	// prevRegular tells whether the heart beat was regular before the newest RR-interval was calculated.
	int qrs, prevRegular;
    
    assert(context != NULL);
    assert(data != NULL && len > 0);
    assert(output != NULL);

    // The main loop where everything proposed in the paper happens. Ends when there are no more signal samples.
    for (c = 0; c < len; ++c)
    {
        // Test if the buffers are full.
        // If they are, shift them, discarding the oldest sample and adding the new one at the end.
        // Else, just put the newest sample in the next free position.
        // Update 'current' so that the program knows where's the newest sample.
		if (context->sample >= context->buffer_size)
		{
			for (i = 0; i < context->buffer_size - 1; i++)
			{
				context->signal[i] = context->signal[i+1];
				context->dcblock[i] = context->dcblock[i+1];
				context->lowpass[i] = context->lowpass[i+1];
				context->highpass[i] = context->highpass[i+1];
				context->derivative[i] = context->derivative[i+1];
				context->squared[i] = context->squared[i+1];
				context->integral[i] = context->integral[i+1];
				context->outputSignal[i] = context->outputSignal[i+1];
			}
			current = context->buffer_size - 1;
		}
		else
		{
			current = context->sample;
		}
		context->signal[current] = data[c];

		context->sample++; // Update sample counter
        
        *output = -1;

		// DC Block filter
		// This was not proposed on the original paper.
		// It is not necessary and can be removed if your sensor or database has no DC noise.
		if (current >= 1)
			context->dcblock[current] = (pan_tompkins_scalar_t)(context->signal[current] - context->signal[current-1] + 0.995*context->dcblock[current-1]);
		else
			context->dcblock[current] = 0;

		// Low Pass filter
		// Implemented as proposed by the original paper.
		// y(nT) = 2y(nT - T) - y(nT - 2T) + x(nT) - 2x(nT - 6T) + x(nT - 12T)
		// Can be removed if your signal was previously filtered, or replaced by a different filter.
		context->lowpass[current] = context->dcblock[current];
		if (current >= 1)
			context->lowpass[current] += 2*context->lowpass[current-1];
		if (current >= 2)
			context->lowpass[current] -= context->lowpass[current-2];
		if (current >= 6)
			context->lowpass[current] -= 2*context->dcblock[current-6];
		if (current >= 12)
			context->lowpass[current] += context->dcblock[current-12];

		// High Pass filter
		// Implemented as proposed by the original paper.
		// y(nT) = 32x(nT - 16T) - [y(nT - T) + x(nT) - x(nT - 32T)]
		// Can be removed if your signal was previously filtered, or replaced by a different filter.
		context->highpass[current] = -context->lowpass[current];
		if (current >= 1)
			context->highpass[current] -= context->highpass[current-1];
		if (current >= 16)
			context->highpass[current] += 32*context->lowpass[current-16];
		if (current >= 32)
			context->highpass[current] += context->lowpass[current-32];

		// Derivative filter
		// This is an alternative implementation, the central difference method.
		// f'(a) = [f(a+h) - f(a-h)]/2h
		// The original formula used by Pan-Tompkins was:
		// y(nT) = (1/8T)[-x(nT - 2T) - 2x(nT - T) + 2x(nT + T) + x(nT + 2T)]
        context->derivative[current] = context->highpass[current];
		if (current > 0)
			context->derivative[current] -= context->highpass[current-1];

		// This just squares the derivative, to get rid of negative values and emphasize high frequencies.
		// y(nT) = [x(nT)]^2.
		context->squared[current] = context->derivative[current]*context->derivative[current];

		// Moving-Window Integration
		// Implemented as proposed by the original paper.
		// y(nT) = (1/N)[x(nT - (N - 1)T) + x(nT - (N - 2)T) + ... x(nT)]
		// WINDOWSIZE, in samples, must be defined so that the window is ~150ms.

		context->integral[current] = 0;
		for (i = 0; i < context->window_size; i++)
		{
			if (current >= i)
				context->integral[current] += context->squared[current - i];
			else
				break;
		}
		context->integral[current] /= (pan_tompkins_scalar_t)i;

		qrs = 0; // false

		// If the current signal is above one of the thresholds (integral or filtered signal), it's a peak candidate.
        if (context->integral[current] >= context->threshold_i1 || context->highpass[current] >= context->threshold_f1)
        {
            peak_i = context->integral[current];
            peak_f = context->highpass[current];
        }

		// If both the integral and the signal are above their thresholds, they're probably signal peaks.
		if ((context->integral[current] >= context->threshold_i1) && (context->highpass[current] >= context->threshold_f1))
		{
			// There's a 200ms latency. If the new peak respects this condition, we can keep testing.
			if (context->sample > context->lastQRS + context->fs/5)
			{
			    // If it respects the 200ms latency, but it doesn't respect the 360ms latency, we check the slope.
				if (context->sample <= context->lastQRS + (size_t)(0.36*context->fs))
				{
				    // The squared slope is "M" shaped. So we have to check nearby samples to make sure we're really looking
				    // at its peak value, rather than a low one.
				    currentSlope = 0;
				    for (j = current - 10; j <= current; j++)
                        if (context->squared[j] > currentSlope)
                            currentSlope = context->squared[j];

				    if (currentSlope <= context->lastSlope / 2)
                    {
                        qrs = 0;
                    }
                    else
                    {
                        context->spk_i = (pan_tompkins_scalar_t)(0.125*peak_i + 0.875*context->spk_i);
                        context->threshold_i1 = (pan_tompkins_scalar_t)(context->npk_i + 0.25*(context->spk_i - context->npk_i));
                        context->threshold_i2 = (pan_tompkins_scalar_t)(0.5 * context->threshold_i1);

                        context->spk_f = (pan_tompkins_scalar_t)(0.125*peak_f + 0.875*context->spk_f);
                        context->threshold_f1 = (pan_tompkins_scalar_t)(context->npk_f + 0.25*(context->spk_f - context->npk_f));
                        context->threshold_f2 = (pan_tompkins_scalar_t)(0.5 * context->threshold_f1);

                        context->lastSlope = currentSlope;
                        qrs = 1; // true
                    }
				}
				// If it was above both thresholds and respects both latency periods, it certainly is a R peak.
				else
				{
				    currentSlope = 0;
                    for (j = current - 10; j <= current; j++)
                        if (context->squared[j] > currentSlope)
                            currentSlope = context->squared[j];

                    context->spk_i = (pan_tompkins_scalar_t)(0.125*peak_i + 0.875*context->spk_i);
                    context->threshold_i1 = (pan_tompkins_scalar_t)(context->npk_i + 0.25*(context->spk_i - context->npk_i));
                    context->threshold_i2 = (pan_tompkins_scalar_t)(0.5 * context->threshold_i1);

                    context->spk_f = (pan_tompkins_scalar_t)(0.125*peak_f + 0.875*context->spk_f);
                    context->threshold_f1 = (pan_tompkins_scalar_t)(context->npk_f + 0.25*(context->spk_f - context->npk_f));
                    context->threshold_f2 = (pan_tompkins_scalar_t)(0.5 * context->threshold_f1);

                    context->lastSlope = currentSlope;
                    qrs = 1; // true
				}
			}
			// If the new peak doesn't respect the 200ms latency, it's noise. Update thresholds and move on to the next sample.
			else
            {
                peak_i = (pan_tompkins_scalar_t)(context->integral[current]);
				context->npk_i = (pan_tompkins_scalar_t)(0.125*peak_i + 0.875*context->npk_i);
				context->threshold_i1 = (pan_tompkins_scalar_t)(context->npk_i + 0.25*(context->spk_i - context->npk_i));
				context->threshold_i2 = (pan_tompkins_scalar_t)(0.5 * context->threshold_i1);
				peak_f = context->highpass[current];
				context->npk_f = (pan_tompkins_scalar_t)(0.125*peak_f + 0.875*context->npk_f);
				context->threshold_f1 = (pan_tompkins_scalar_t)(context->npk_f + 0.25*(context->spk_f - context->npk_f));
                context->threshold_f2 = (pan_tompkins_scalar_t)(0.5 * context->threshold_f1);
                qrs = 0; // false
				context->outputSignal[current] = qrs;
				if (context->sample > context->delay + context->buffer_size)
                    *output++ = context->outputSignal[0];
                continue;
            }
		}

		// If a R-peak was detected, the RR-averages must be updated.
		if (qrs)
		{
			// Add the newest RR-interval to the buffer and get the new average.
			context->rravg1 = 0;
			for (i = 0; i < 7; i++)
			{
				context->rr1[i] = context->rr1[i+1];
				context->rravg1 += context->rr1[i];
			}
			context->rr1[7] = (double)(context->sample - context->lastQRS);
			context->lastQRS = context->sample;
			context->rravg1 += context->rr1[7];
			context->rravg1 = context->rravg1 * 0.125;

			// If the newly-discovered RR-average is normal, add it to the "normal" buffer and get the new "normal" average.
			// Update the "normal" beat parameters.
			if ( (context->rr1[7] >= context->rrlow) && (context->rr1[7] <= context->rrhigh) )
			{
				context->rravg2 = 0;
				for (i = 0; i < 7; i++)
				{
					context->rr2[i] = context->rr2[i+1];
					context->rravg2 += context->rr2[i];
				}
				context->rr2[7] = context->rr1[7];
				context->rravg2 += context->rr2[7];
				context->rravg2 = context->rravg2 * 0.125;
				context->rrlow = 0.92*context->rravg2;
				context->rrhigh = 1.16*context->rravg2;
				context->rrmiss = 1.66*context->rravg2;
			}

			prevRegular = context->regular;
			if (context->rravg1 == context->rravg2)
			{
				context->regular = 1; // true
			}
			// If the beat had been normal but turned odd, change the thresholds.
			else
			{
				context->regular = 0; // true
				if (prevRegular)
				{
					context->threshold_i1 /= 2;
					context->threshold_f1 /= 2;
				}
			}
		}
		// If no R-peak was detected, it's important to check how long it's been since the last detection.
		else
		{
		    // If no R-peak was detected for too long, use the lighter thresholds and do a back search.
			// However, the back search must respect the 200ms limit and the 360ms one (check the slope).
			if ((context->sample - context->lastQRS > (long unsigned int)context->rrmiss) && (context->sample > context->lastQRS + context->fs/5))
			{
				for (i = current - (context->sample - context->lastQRS) + context->fs/5; i < (long unsigned int)current; i++)
				{
					if ( (context->integral[i] > context->threshold_i2) && (context->highpass[i] > context->threshold_f2))
					{
					    currentSlope = 0;
                        for (j = i - 10; j <= i; j++)
                            if (context->squared[j] > currentSlope)
                                currentSlope = context->squared[j];

                        if ((currentSlope < context->lastSlope / 2) && (i + context->sample) < context->lastQRS + 0.36*context->lastQRS)
                        {
                            qrs = 0; // false
                        }
                        else
                        {
                            peak_i = context->integral[i];
                            peak_f = context->highpass[i];
                            context->spk_i = (pan_tompkins_scalar_t)(0.25*peak_i+ 0.75*context->spk_i);
                            context->spk_f = (pan_tompkins_scalar_t)(0.25*peak_f + 0.75*context->spk_f);
                            context->threshold_i1 = (pan_tompkins_scalar_t)(context->npk_i + 0.25*(context->spk_i - context->npk_i));
                            context->threshold_i2 = (pan_tompkins_scalar_t)(0.5 * context->threshold_i1);
                            context->lastSlope = currentSlope;
                            context->threshold_f1 = (pan_tompkins_scalar_t)(context->npk_f + 0.25*(context->spk_f - context->npk_f));
                            context->threshold_f2 = (pan_tompkins_scalar_t)(0.5 * context->threshold_f1);
                            // If a signal peak was detected on the back search, the RR attributes must be updated.
                            // This is the same thing done when a peak is detected on the first try.
                            //RR Average 1
                            context->rravg1 = 0;
                            for (j = 0; j < 7; j++)
                            {
                                context->rr1[j] = context->rr1[j+1];
                                context->rravg1 += context->rr1[j];
                            }
                            context->rr1[7] = (double)(context->sample - (current - i) - context->lastQRS);
                            qrs = 1; // true
                            context->lastQRS = context->sample - (current - i);
                            context->rravg1 += context->rr1[7];
                            context->rravg1 = context->rravg1 * 0.125;

                            //RR Average 2
                            if ( (context->rr1[7] >= context->rrlow) && (context->rr1[7] <= context->rrhigh) )
                            {
                                context->rravg2 = 0;
                                for (i = 0; i < 7; i++)
                                {
                                    context->rr2[i] = context->rr2[i+1];
                                    context->rravg2 += context->rr2[i];
                                }
                                context->rr2[7] = context->rr1[7];
                                context->rravg2 += context->rr2[7];
                                context->rravg2 = context->rravg2 * 0.125;
                                context->rrlow = 0.92*context->rravg2;
                                context->rrhigh = 1.16*context->rravg2;
                                context->rrmiss = 1.66*context->rravg2;
                            }

                            prevRegular = context->regular;
                            if (context->rravg1 == context->rravg2)
                            {
                                context->regular = 1; // true
                            }
                            else
                            {
                                context->regular = 0; // false
                                if (prevRegular)
                                {
                                    context->threshold_i1 /= 2;
                                    context->threshold_f1 /= 2;
                                }
                            }

                            break;
                        }
                    }
				}

				if (qrs)
                {
                    context->outputSignal[current] = 0; // false
                    context->outputSignal[i] = 1; // true
                    if (context->sample > context->delay + context->buffer_size)
                        *output++ = context->outputSignal[0];
                    continue;
                }
			}

			// Definitely no signal peak was detected.
			if (!qrs)
			{
				// If some kind of peak had been detected, then it's certainly a noise peak. Thresholds must be updated accordinly.
				if ((context->integral[current] >= context->threshold_i1) || (context->highpass[current] >= context->threshold_f1))
				{
					peak_i = context->integral[current];
					context->npk_i = (pan_tompkins_scalar_t)(0.125*peak_i + 0.875*context->npk_i);
					context->threshold_i1 = (pan_tompkins_scalar_t)(context->npk_i + 0.25*(context->spk_i - context->npk_i));
					context->threshold_i2 = (pan_tompkins_scalar_t)(0.5 * context->threshold_i1);
					peak_f = context->highpass[current];
					context->npk_f = (pan_tompkins_scalar_t)(0.125*peak_f + 0.875*context->npk_f);
					context->threshold_f1 = (pan_tompkins_scalar_t)(context->npk_f + 0.25*(context->spk_f - context->npk_f));
					context->threshold_f2 = (pan_tompkins_scalar_t)(0.5 * context->threshold_f1);
				}
			}
		}
		// The current implementation outputs '0' for every sample where no peak was detected,
		// and '1' for every sample where a peak was detected. It should be changed to fit
		// the desired application.
		// The 'if' accounts for the delay introduced by the filters: we only start outputting after the delay.
		// However, it updates a few samples back from the buffer. The reason is that if we update the detection
		// for the current sample, we might miss a peak that could've been found later by backsearching using
		// lighter thresholds. The final waveform output does match the original signal, though.
		context->outputSignal[current] = qrs;
		if (context->sample > context->delay + context->buffer_size)
			*output++ = context->outputSignal[0];
	}

	// Output the last remaining samples on the buffer
	//for (i = 1; i < context->buffer_size; i++)
	//	output(context->outputSignal[i]);
}


#define FREE_X_(ptr) \
    do \
    { \
        if ((ptr) != NULL) \
        { \
            free((ptr)); \
            (ptr) = NULL; \
        } \
    } while (0)


void pan_tompkins_free(pan_tompkins_context_t *context)
{
    if (context != NULL && *context != NULL)
    {
        FREE_X_((*context)->signal);
        FREE_X_((*context)->dcblock);
        FREE_X_((*context)->lowpass);
        FREE_X_((*context)->highpass);
        FREE_X_((*context)->derivative);
        FREE_X_((*context)->squared);
        FREE_X_((*context)->integral);
        FREE_X_((*context)->outputSignal);
        
        FREE_X_(*context);
    }
}
