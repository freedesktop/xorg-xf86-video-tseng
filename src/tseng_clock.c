
/* $XFree86: xc/programs/Xserver/hw/xfree86/drivers/tseng/tseng_clock.c,v 1.18 2003/11/03 05:11:44 tsi Exp $ */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

/*
 *
 * Copyright 1993-1997 The XFree86 Project, Inc.
 *
 */
/**
 ** Clock setting methods for Tseng chips
 **
 ** The *ClockSelect() fucntions are ONLY used used for clock probing!
 ** Setting the actual clock is done in TsengRestore().
 **/

#include "tseng.h"

void tseng_clock_setup(ScrnInfoPtr pScrn)
{
    TsengPtr pTseng = TsengPTR(pScrn);
    int dacspeed, mem_bw;

    PDEBUG("	tseng_clock_setup\n");

    /*
     * Memory bandwidth is important in > 8bpp modes, especially on ET4000
     *
     * This code evaluates a video mode with respect to requested dot clock
     * (depends on the VGA chip and the RAMDAC) and the resulting bandwidth
     * demand on memory (which in turn depends on color depth).
     *
     * For each mode, the minimum of max data transfer speed (dot clock
     * limit) and memory bandwidth determines if the mode is allowed.
     *
     * We should also take acceleration into account: accelerated modes
     * strain the bandwidth heavily, because they cause lots of random
     * acesses to video memory, which is bad for bandwidth due to smaller
     * page-mode memory requests.
     */

    /* Set the min pixel clock */
    pTseng->MinClock = 12000;	       /* XXX Guess, need to check this */

    /* default */
    dacspeed = MAX_TSENG_CLOCK;
    /*
     * According to Tseng (about the ET6000):
     * "Besides the 135 MHz maximum pixel clock frequency, the other limit has to
     * do with where you get FIFO breakdown (usually appears as stray horizontal
     * lines on the screen). Assuming the accelerator is running steadily doing a
     * worst case operation, to avoid FIFO breakdown you should keep the product
     *   pixel_clock*(bytes/pixel) <= 225 MHz . This is based on an XCLK
     * (system/memory) clock of 92 MHz (which is what we currently use) and
     * a value in the RAS/CAS Configuration register (CFG 44) of either 015h
     * or 014h (depending on the type of MDRAM chips). Also, the FIFO low
     * threshold control bit (bit 4 of CFG 41) should be set for modes where
     * pixel_clock*(bytes/pixel) > 130 MHz . These limits are for the
     * current ET6000 chips. The ET6100 will raise the pixel clock limit
     * to 175 MHz and the pixel_clock*(bytes/pixel) FIFO breakdown limit
     * to about 275 MHz."
     */
    if (pTseng->ChipType == ET6000) {
        if (pTseng->ChipRev == REV_ET6100) {
            dacspeed = 175000;
            mem_bw = 280000; /* 275000 is _just_ not enough for 1152x864x24 @ 70Hz */
        } else { /* ET6000 */
            dacspeed = 135000;
            mem_bw = 225000;
        }
    } else {
        if (pScrn->bitsPerPixel == 8)
            dacspeed = 135000; /* we can do PIXMUX */
        else
            dacspeed = MAX_TSENG_CLOCK;
        
        mem_bw	= 90000;
        if (pScrn->videoRam > 1024)
            mem_bw = 150000; /* interleaved DRAM gives 70% more bandwidth */
    }
    pTseng->max_vco_freq = dacspeed*2+1;

    /*
     * "dacspeed" is the theoretical limit imposed by the RAMDAC.
     * "mem_bw" is the max memory bandwidth in mb/sec available
     * for the pixel FIFO.
     * The lowest of the two determines the actual pixel clock limit.
     */
    dacspeed = min(dacspeed, (mem_bw / pTseng->Bytesperpixel));


    /*
     * Setup the ClockRanges, which describe what clock ranges are available,
     * and what sort of modes they can be used for.
     *
     * First, we set up the default case, and modify it later if needed.
     */
    pTseng->clockRange[0] = xnfcalloc(sizeof(ClockRange), 1);
    pTseng->clockRange[0]->next = NULL;
    pTseng->clockRange[0]->minClock = pTseng->MinClock;
    pTseng->clockRange[0]->maxClock = dacspeed;
    pTseng->clockRange[0]->clockIndex = -1;      /* programmable -- not used */
    pTseng->clockRange[0]->interlaceAllowed = TRUE;
    pTseng->clockRange[0]->doubleScanAllowed = TRUE;
    pTseng->clockRange[0]->ClockMulFactor = 1;
    pTseng->clockRange[0]->ClockDivFactor = 1;
    pTseng->clockRange[0]->PrivFlags = TSENG_MODE_NORMAL;
    
    if (pTseng->ChipType == ET4000) {
        /*
         * Handle PIXMUX modes.
         *
         * NOTE: We disable PIXMUX when clockchip programming on the GenDAC
         * family is disabled. PIXMUX requires that the N2 post-divider in the
         * PLL clock programming word is >= 2, which is not always true for the
         * default (BIOS) clocks programmed in the 8 clock registers.
         */
        if (pScrn->bitsPerPixel == 8) {
            pTseng->clockRange[0]->maxClock = MAX_TSENG_CLOCK;
            /* set up 2nd clock range for PIXMUX modes */
            pTseng->clockRange[1] = xnfcalloc(sizeof(ClockRange), 1);
            pTseng->clockRange[0]->next = pTseng->clockRange[1];
            pTseng->clockRange[1]->next = NULL;
            pTseng->clockRange[1]->minClock = 75000;
            pTseng->clockRange[1]->maxClock = dacspeed;
            pTseng->clockRange[1]->clockIndex = -1;      /* programmable -- not used */
            pTseng->clockRange[1]->interlaceAllowed = TRUE;
            pTseng->clockRange[1]->doubleScanAllowed = TRUE;
            pTseng->clockRange[1]->ClockMulFactor = 1;
            pTseng->clockRange[1]->ClockDivFactor = 2;
            pTseng->clockRange[1]->PrivFlags = TSENG_MODE_PIXMUX;
        }

        /*
         * Handle 16/24/32 bpp modes that require some form of clock scaling. We
         * can have either 8-bit DACs that require "bytesperpixel" clocks per
         * pixel, or 16-bit DACs that can transport 8 or 16 bits per clock.
         */
        if (pTseng->Bytesperpixel > 1) {
            /* in either 8 or 16-bit DAC case, we can use an 8-bit interface */
            pTseng->clockRange[0]->maxClock = min(MAX_TSENG_CLOCK / pTseng->Bytesperpixel, dacspeed);
            pTseng->clockRange[0]->ClockMulFactor = pTseng->Bytesperpixel;
            pTseng->clockRange[0]->ClockDivFactor = 1;
            /* in addition, 16-bit DACs can also transport 2 bytes per clock */
            pTseng->clockRange[1] = xnfcalloc(sizeof(ClockRange), 1);
            pTseng->clockRange[0]->next = pTseng->clockRange[1];
            pTseng->clockRange[1]->next = NULL;
            pTseng->clockRange[1]->minClock = pTseng->MinClock;
            pTseng->clockRange[1]->maxClock = min((MAX_TSENG_CLOCK * 2) / pTseng->Bytesperpixel, dacspeed);
            pTseng->clockRange[1]->clockIndex = -1;      /* programmable -- not used */
            pTseng->clockRange[1]->interlaceAllowed = TRUE;
            pTseng->clockRange[1]->doubleScanAllowed = TRUE;
            pTseng->clockRange[1]->ClockMulFactor = pTseng->Bytesperpixel;
            pTseng->clockRange[1]->ClockDivFactor = 2;
            pTseng->clockRange[1]->PrivFlags = TSENG_MODE_DACBUS16;
        }
    }

    if (pTseng->clockRange[1])
	pTseng->MaxClock = pTseng->clockRange[1]->maxClock;
    else
	pTseng->MaxClock = pTseng->clockRange[0]->maxClock;

    xf86DrvMsg(pScrn->scrnIndex, X_DEFAULT, "Min pixel clock is %d MHz\n",
	pTseng->MinClock / 1000);
    xf86DrvMsg(pScrn->scrnIndex, X_PROBED, "Max pixel clock is %d MHz\n",
	pTseng->MaxClock / 1000);
}

#define BASE_FREQ         14.31818     /* MHz */
void
TsengcommonCalcClock(long freq, int min_m, int min_n1, int max_n1, int min_n2, int max_n2,
    long freq_min, long freq_max,
    unsigned char *mdiv, unsigned char *ndiv)
{
    double ffreq, ffreq_min, ffreq_max;
    double div, diff, best_diff;
    unsigned int m;
    unsigned char n1, n2;
    unsigned char best_n1 = 16 + 2, best_n2 = 2, best_m = 125 + 2;

    PDEBUG("	commonCalcClock\n");
    ffreq = freq / 1000.0 / BASE_FREQ;
    ffreq_min = freq_min / 1000.0 / BASE_FREQ;
    ffreq_max = freq_max / 1000.0 / BASE_FREQ;

    if (ffreq < ffreq_min / (1 << max_n2)) {
	ErrorF("invalid frequency %1.3f MHz  [freq >= %1.3f MHz]\n",
	    ffreq * BASE_FREQ, ffreq_min * BASE_FREQ / (1 << max_n2));
	ffreq = ffreq_min / (1 << max_n2);
    }
    if (ffreq > ffreq_max / (1 << min_n2)) {
	ErrorF("invalid frequency %1.3f MHz  [freq <= %1.3f MHz]\n",
	    ffreq * BASE_FREQ, ffreq_max * BASE_FREQ / (1 << min_n2));
	ffreq = ffreq_max / (1 << min_n2);
    }
    /* work out suitable timings */

    best_diff = ffreq;

    for (n2 = min_n2; n2 <= max_n2; n2++) {
	for (n1 = min_n1 + 2; n1 <= max_n1 + 2; n1++) {
	    m = (int)(ffreq * n1 * (1 << n2) + 0.5);
	    if (m < min_m + 2 || m > 127 + 2)
		continue;
	    div = (double)(m) / (double)(n1);
	    if ((div >= ffreq_min) &&
		(div <= ffreq_max)) {
		diff = ffreq - div / (1 << n2);
		if (diff < 0.0)
		    diff = -diff;
		if (diff < best_diff) {
		    best_diff = diff;
		    best_m = m;
		    best_n1 = n1;
		    best_n2 = n2;
		}
	    }
	}
    }

#ifdef EXTENDED_DEBUG
    ErrorF("Clock parameters for %1.6f MHz: m=%d, n1=%d, n2=%d\n",
	((double)(best_m) / (double)(best_n1) / (1 << best_n2)) * BASE_FREQ,
	best_m - 2, best_n1 - 2, best_n2);
#endif

    if (max_n1 == 63)
	*ndiv = (best_n1 - 2) | (best_n2 << 6);
    else
	*ndiv = (best_n1 - 2) | (best_n2 << 5);
    *mdiv = best_m - 2;
}

