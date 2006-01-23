/*
 * Copyright 2005-2006 Luc Verhaegen.
 * Copyright 1993-1997 The XFree86 Project, Inc.
 * Copyright 1990-1991 Thomas Roell.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sub license,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "tseng.h"

/*
 * lacking from hwp
 */
static CARD8
vgaHWReadDacWriteAddr(vgaHWPtr hwp)
{
    if (hwp->MMIOBase)
	return MMIO_IN8(hwp->MMIOBase, hwp->MMIOOffset + VGA_DAC_WRITE_ADDR);
    else
	return inb(hwp->PIOOffset + VGA_DAC_WRITE_ADDR);
}

/*
 *
 * RAMDAC handling.
 *
 */

/*
 *
 */
static Bool
tsengSTG170xDetect(ScrnInfoPtr pScrn)
{
    TsengPtr pTseng = TsengPTR(pScrn);
    vgaHWPtr hwp = VGAHWPTR(pScrn);
    CARD8 temp, cid, did, readDacMask;  

    /* TSENGFUNC(pScrn->scrnIndex); */

    hwp->writeDacWriteAddr(hwp, 0x00);
    readDacMask = hwp->readDacMask(hwp);
    hwp->readDacMask(hwp);
    hwp->readDacMask(hwp);
    hwp->readDacMask(hwp);
    temp = hwp->readDacMask(hwp);

    hwp->writeDacWriteAddr(hwp, 0x00);
    hwp->readDacMask(hwp);
    hwp->readDacMask(hwp);
    hwp->readDacMask(hwp);
    hwp->readDacMask(hwp);
    hwp->writeDacMask(hwp, temp | 0x10);

    hwp->writeDacWriteAddr(hwp, 0x00);
    hwp->readDacMask(hwp);
    hwp->readDacMask(hwp);
    hwp->readDacMask(hwp);
    hwp->readDacMask(hwp);
    hwp->readDacMask(hwp);
    hwp->writeDacMask(hwp, 0x00);
    hwp->writeDacMask(hwp, 0x00);
    cid = hwp->readDacMask(hwp); /* company ID */
    did = hwp->readDacMask(hwp); /* device ID */

    hwp->writeDacWriteAddr(hwp, 0x00);
    hwp->readDacMask(hwp);
    hwp->readDacMask(hwp);
    hwp->readDacMask(hwp);
    hwp->readDacMask(hwp);
    hwp->writeDacMask(hwp, temp);

    hwp->writeDacWriteAddr(hwp, 0x00);
    hwp->writeDacMask(hwp, readDacMask);
    
    hwp->writeDacWriteAddr(hwp, 0x00);

    if ((cid == 0x44) && (did == 0x03))	{
        xf86DrvMsg(pScrn->scrnIndex, X_PROBED, "Detected STG-1703 RAMDAC.\n");
        pTseng->RAMDAC = STG1703;
        return TRUE;
    }
    return FALSE;
}

/*
 *
 */
static Bool
tsengCH8398Detect(ScrnInfoPtr pScrn)
{
    TsengPtr pTseng = TsengPTR(pScrn);
    vgaHWPtr hwp = VGAHWPTR(pScrn);
    CARD8 temp;

    /* TSENGFUNC(pScrn->scrnIndex); */

    hwp->writeDacWriteAddr(hwp, 0x00);
    hwp->readDacMask(hwp);
    hwp->readDacMask(hwp);
    hwp->readDacMask(hwp);
    temp = hwp->readDacMask(hwp);

    if (temp == 0xC0) {
        xf86DrvMsg(pScrn->scrnIndex, X_PROBED, "Detected Chrontel 8398 RAMDAC.\n");
        pTseng->RAMDAC = CH8398;
        return TRUE;
    }
    return FALSE;
}

/*
 *
 */
Bool
TsengRAMDACProbe(ScrnInfoPtr pScrn)
{
    TsengPtr pTseng = TsengPTR(pScrn);

    PDEBUG("	Check_Tseng_Ramdac\n");

    if (pTseng->ChipType == ET6000) {
        int mclk;
        int dbyte;

        /* There are some limits here though: 80000 <= MemClk <= 110000 */
        (void) inb(pTseng->IOAddress + 0x67);
        outb(pTseng->IOAddress + 0x67, 10);
        mclk = (inb(pTseng->IOAddress + 0x69) + 2) * 14318;
        dbyte = inb(pTseng->IOAddress + 0x69);
        mclk /= ((dbyte & 0x1f) + 2) * (1 << ((dbyte >> 5) & 0x03));
        pTseng->MemClk = mclk;

    } else { /* ET4000W32P has external ramdacs */
        vgaHWPtr hwp = VGAHWPTR(pScrn);
        CARD8 CR;

        vgaHWReadDacWriteAddr(hwp);
        hwp->readDacMask(hwp);
        hwp->readDacMask(hwp);
        hwp->readDacMask(hwp);
        hwp->readDacMask(hwp);
        CR = hwp->readDacMask(hwp);
        vgaHWReadDacWriteAddr(hwp);

        if (!tsengSTG170xDetect(pScrn) && !tsengCH8398Detect(pScrn)) {
            xf86DrvMsg(pScrn->scrnIndex, X_ERROR, "%s: Unable to probe RAMDAC\n",
                       __func__);
            return FALSE;
        }

        vgaHWReadDacWriteAddr(hwp);
        hwp->readDacMask(hwp);
        hwp->readDacMask(hwp);
        hwp->readDacMask(hwp);
        hwp->readDacMask(hwp);
        hwp->writeDacMask(hwp, CR);
        vgaHWReadDacWriteAddr(hwp);

        hwp->writeDacMask(hwp, 0xFF);
    }

    return TRUE;
}

/*
 * This sets up the RAMDAC registers for the correct BPP and pixmux values.
 * (also set VGA controller registers for pixmux and BPP)
 */
static void
tseng_set_ramdac_bpp(ScrnInfoPtr pScrn, DisplayModePtr mode)
{
    TsengPtr pTseng = TsengPTR(pScrn);
    
    if (pTseng->ChipType == ET6000) {
        /* ATC index 0x16 -- bits-per-PCLK */
        pTseng->ModeReg.ExtATC &= 0xCF;
        pTseng->ModeReg.ExtATC |= (pTseng->Bytesperpixel - 1) << 4;

        if (pScrn->bitsPerPixel == 15)
            pTseng->ModeReg.ET6K_58 &= ~0x02; /* 5-5-5 RGB mode */
        else if (pScrn->bitsPerPixel == 16)
            pTseng->ModeReg.ET6K_58 |= 0x02; /* 5-6-5 RGB mode */
    } else {
        Bool PixMux = FALSE;

        if ((mode->PrivFlags == TSENG_MODE_DACBUS16) ||
            (mode->PrivFlags == TSENG_MODE_PIXMUX))
            PixMux = TRUE;

        /* ATC index 0x16 -- bits-per-PCLK */
        pTseng->ModeReg.ExtATC &= 0xCF;
        pTseng->ModeReg.ExtATC |= 0x20;

        switch (pTseng->RAMDAC) {
        case STG1703:
            pTseng->ModeReg.pll.cmd_reg &= 0x04; /* keep 7.5 IRE setup setting */
            pTseng->ModeReg.pll.cmd_reg |= 0x18; /* enable ext regs and pixel modes */

            switch (pScrn->bitsPerPixel) {
            case 8:
                if (PixMux)
                    pTseng->ModeReg.pll.ctrl = 0x05;
                else
                    pTseng->ModeReg.pll.ctrl = 0x00;
                break;
            case 15:
                if (PixMux)
                    pTseng->ModeReg.pll.ctrl = 0x02;
                else
                    pTseng->ModeReg.pll.ctrl = 0x08;
                pTseng->ModeReg.pll.cmd_reg |= 0xA0;
                break;
            /* !!!! We only should do pixmux from here on end !!!! */  
            case 16:
                pTseng->ModeReg.pll.ctrl = 0x03;
                pTseng->ModeReg.pll.cmd_reg |= 0xC0;
                break;
            case 24:
                pTseng->ModeReg.pll.ctrl = 0x09;
                pTseng->ModeReg.pll.cmd_reg |= 0xE0;
                break;
            case 32:
                pTseng->ModeReg.pll.ctrl = 0x04;
                pTseng->ModeReg.pll.cmd_reg |= 0xE0;
                break;
            default:
                pTseng->ModeReg.pll.ctrl = 0;
                xf86DrvMsg(pScrn->scrnIndex, X_ERROR, "%s: STG1703 RAMDAC doesn't"
                           " support %dbpp.\n", __func__, pScrn->bitsPerPixel);
            }

            /* set PLL (input) range */
            if (mode->SynthClock <= 16000)
                pTseng->ModeReg.pll.timingctrl = 0;
            else if (mode->SynthClock <= 32000)
                pTseng->ModeReg.pll.timingctrl = 1;
            else if (mode->SynthClock <= 67500)
                pTseng->ModeReg.pll.timingctrl = 2;
            else
                pTseng->ModeReg.pll.timingctrl = 3;

            break;
        case CH8398:
            switch (pScrn->bitsPerPixel) {
            case 8:
                if (PixMux)
                    pTseng->ModeReg.pll.cmd_reg = 0x24;
                else
                    pTseng->ModeReg.pll.cmd_reg = 0x04;
                break;
            case 15:
                if (PixMux)
                    pTseng->ModeReg.pll.cmd_reg = 0x14;
                else
                    pTseng->ModeReg.pll.cmd_reg = 0xC4;
                break;
            case 16:
                if (PixMux)
                    pTseng->ModeReg.pll.cmd_reg = 0x34;
                else
                    pTseng->ModeReg.pll.cmd_reg = 0x64;
                break;
            case 24:
                if (PixMux)
                    pTseng->ModeReg.pll.cmd_reg = 0xB4;
                else
                    pTseng->ModeReg.pll.cmd_reg = 0x74;
                break;
            default:
                pTseng->ModeReg.pll.cmd_reg = 0;
                xf86DrvMsg(pScrn->scrnIndex, X_ERROR, "%s: CH8398 RAMDAC doesn't"
                           " support %dbpp.\n", __func__, pScrn->bitsPerPixel);
            }
            break;
        default:
            return;
        }
    }
#ifdef FIXME /* still needed? */
    if (mode->PrivFlags == TSENG_MODE_PIXMUX) {
	VGAHWPTR(pScrn)->ModeReg.CRTC[0x17] &= 0xFB;

	/* to avoid blurred vertical line during flyback, disable H-blanking
	 * (better solution needed !!!)
	 */
	VGAHWPTR(pScrn)->ModeReg.CRTC[0x02] = 0xff;
    }
#endif
}

/*
 *
 */
void
tseng_clock_setup(ScrnInfoPtr pScrn)
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

/*
 *
 */
#define BASE_FREQ         14.31818     /* MHz */
static void
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

/*
 * adjust the current video frame (viewport) to display the mousecursor.
 */
void
TsengAdjustFrame(int scrnIndex, int x, int y, int flags)
{
    ScrnInfoPtr pScrn = xf86Screens[scrnIndex];
    TsengPtr pTseng = TsengPTR(pScrn);
    int iobase = VGAHWPTR(pScrn)->IOBase;
    int Base;

    PDEBUG("	TsengAdjustFrame\n");

    if (pTseng->ShowCache) {
	if (y)
	    y += 256;
    }
    if (pScrn->bitsPerPixel < 8)
	Base = (y * pScrn->displayWidth + x + 3) >> 3;
    else {
	Base = ((y * pScrn->displayWidth + x + 1) * pTseng->Bytesperpixel) >> 2;
	/* adjust Base address so it is a non-fractional multiple of pTseng->Bytesperpixel */
	Base -= (Base % pTseng->Bytesperpixel);
    }

    outw(iobase + 4, (Base & 0x00FF00) | 0x0C);
    outw(iobase + 4, ((Base & 0x00FF) << 8) | 0x0D);
    outw(iobase + 4, ((Base & 0x0F0000) >> 8) | 0x33);

}

/*
 *
 */
ModeStatus
TsengValidMode(int scrnIndex, DisplayModePtr mode, Bool verbose, int flags)
{

    PDEBUG("	TsengValidMode\n");

#ifdef FIXME
  is this needed? xf86ValidMode gets HMAX and VMAX variables, so it could deal with this.
  need to recheck hsize with mode->Htotal*mulFactor/divFactor
    /* Check for CRTC timing bits overflow. */
    if (mode->HTotal > Tseng_HMAX) {
	return MODE_BAD_HVALUE;
    }
    if (mode->VTotal > Tseng_VMAX) {
	return MODE_BAD_VVALUE;
    }
#endif

    return MODE_OK;
}

/*
 * Save the current video mode
 */
void
TsengSave(ScrnInfoPtr pScrn)
{
    unsigned char temp, saveseg1 = 0, saveseg2 = 0;
    TsengPtr pTseng = TsengPTR(pScrn);
    vgaRegPtr vgaReg;
    TsengRegPtr tsengReg;
    int iobase = VGAHWPTR(pScrn)->IOBase;

    PDEBUG("	TsengSave\n");

    vgaReg = &VGAHWPTR(pScrn)->SavedReg;
    tsengReg = &pTseng->SavedReg;

    /*
     * This function will handle creating the data structure and filling
     * in the generic VGA portion.
     */
    vgaHWSave(pScrn, vgaReg, VGA_SR_ALL);

    /*
     * we need this here , cause we MUST disable the ROM SYNC feature
     * this bit changed with W32p_rev_c...
     */
    outb(iobase + 4, 0x34);
    temp = inb(iobase + 5);
    tsengReg->CR34 = temp;
    if ((pTseng->ChipType == ET4000) &&
        ((pTseng->ChipRev == REV_A) || (pTseng->ChipRev == REV_B))) {
#ifdef OLD_CODE
	outb(iobase + 5, temp & 0x1F);
#else
	/* data books say translation ROM is controlled by bits 4 and 5 */
	outb(iobase + 5, temp & 0xCF);
#endif
    }

    saveseg1 = inb(0x3CD);
    outb(0x3CD, 0x00);		       /* segment select 1 */

    saveseg2 = inb(0x3CB);
    outb(0x3CB, 0x00);	       /* segment select 2 */

    tsengReg->ExtSegSel[0] = saveseg1;
    tsengReg->ExtSegSel[1] = saveseg2;

    outb(iobase + 4, 0x33);
    tsengReg->CR33 = inb(iobase + 5);
    outb(iobase + 4, 0x35);
    tsengReg->CR35 = inb(iobase + 5);
    if (pTseng->ChipType == ET4000) {
	outb(iobase + 4, 0x36);
	tsengReg->CR36 = inb(iobase + 5);
	outb(iobase + 4, 0x37);
	tsengReg->CR37 = inb(iobase + 5);
	outb(0x217a, 0xF7);
	tsengReg->ExtIMACtrl = inb(0x217b);

	outb(iobase + 4, 0x32);
	tsengReg->CR32 = inb(iobase + 5);
    }
    outb(0x3C4, 6);
    tsengReg->SR06 = inb(0x3C5);
    outb(0x3C4, 7);
    tsengReg->SR07 = inb(0x3C5);
    tsengReg->SR07 |= 0x14;
    temp = inb(iobase + 0x0A);	       /* reset flip-flop */
    outb(0x3C0, 0x36);
    tsengReg->ExtATC = inb(0x3C1);
    outb(0x3C0, tsengReg->ExtATC);

    if (pTseng->ChipType == ET4000) {
        switch (pTseng->RAMDAC) {
        case STG1703:
#ifdef TODO
            /* Save STG 1703 GenDAC Command and PLL registers 
             * unfortunately we reuse the gendac data structure, so the 
             * field names are not really good.
             */
            
            outb(0x3C8, 0);
            (void)inb(0x3C6);
            (void)inb(0x3C6);
            (void)inb(0x3C6);
            (void)inb(0x3C6);
            tsengReg->pll.cmd_reg = inb(0x3C6); /* Enhanced command register */
            outb(0x3C8, 0);

            tsengReg->pll.f2_M = STG1703getIndex(0x24);		/* f2 PLL M divider */
            tsengReg->pll.f2_N = inb(0x3c6);	/* f2 PLL N1/N2 divider */

            tsengReg->pll.ctrl = STG1703getIndex(0x03);	/* pixel mode select control */
            tsengReg->pll.timingctrl = STG1703getIndex(0x05);	/* pll timing control */
#else
             xf86DrvMsg(pScrn->scrnIndex, X_WARNING, "Not implemented!\n");
#endif
            break;
        case CH8398:
            outb(0x3C8, 0);
            (void)inb(0x3C6);
            (void)inb(0x3C6);
            (void)inb(0x3C6);
            (void)inb(0x3C6);
            tsengReg->pll.cmd_reg = inb(0x3C6); /* Enhanced command register */
            outb(0x3C8, 0);

            inb(0x3c8);
            inb(0x3c6);
            inb(0x3c6);
            inb(0x3c6);
            inb(0x3c6);
            inb(0x3c6);
            tsengReg->pll.timingctrl = inb(0x3c6);
            /* Save PLL */
            outb(iobase + 4, 0x31);
            temp = inb(iobase + 5);
            outb(iobase + 5, temp | (1 << 6));	/* set RS2 through CS3 */
            /* We are in ClockRAM mode 0x3c7 = CRA, 0x3c8 = CWA, 0x3c9 = CDR */
            tsengReg->pll.r_idx = inb(0x3c7);
            tsengReg->pll.w_idx = inb(0x3c8);
            outb(0x3c7, 10);
            tsengReg->pll.f2_N = inb(0x3c9);
            tsengReg->pll.f2_M = inb(0x3c9);
            outb(0x3c7, tsengReg->pll.r_idx);
            inb(0x3c8);		       /* loop to Clock Select Register */
            inb(0x3c8);
            inb(0x3c8);
            inb(0x3c8);
            tsengReg->pll.ctrl = inb(0x3c8);
            outb(iobase + 4, 0x31);
            outb(iobase + 5, temp);
            break;
        default:
            break;
	}
    } else {
	/* Save ET6000 CLKDAC PLL registers */
	temp = inb(pTseng->IOAddress + 0x67);	/* remember old CLKDAC index register pointer */
	outb(pTseng->IOAddress + 0x67, 2);
	tsengReg->pll.f2_M = inb(pTseng->IOAddress + 0x69);
	tsengReg->pll.f2_N = inb(pTseng->IOAddress + 0x69);
	/* save MClk values */
	outb(pTseng->IOAddress + 0x67, 10);
	tsengReg->pll.MClkM = inb(pTseng->IOAddress + 0x69);
	tsengReg->pll.MClkN = inb(pTseng->IOAddress + 0x69);
	/* restore old index register */
	outb(pTseng->IOAddress + 0x67, temp);
    }

    if (pTseng->ChipType == ET6000) {
	tsengReg->ET6K_13 = inb(pTseng->IOAddress + 0x13);
	tsengReg->ET6K_40 = inb(pTseng->IOAddress + 0x40);
	tsengReg->ET6K_58 = inb(pTseng->IOAddress + 0x58);
	tsengReg->ET6K_41 = inb(pTseng->IOAddress + 0x41);
	tsengReg->ET6K_44 = inb(pTseng->IOAddress + 0x44);
	tsengReg->ET6K_46 = inb(pTseng->IOAddress + 0x46);
    }
    outb(iobase + 4, 0x30);
    tsengReg->CR30 = inb(iobase + 5);
    outb(iobase + 4, 0x31);
    tsengReg->CR31 = inb(iobase + 5);
    outb(iobase + 4, 0x3F);
    tsengReg->CR3F = inb(iobase + 5);
}

/*
 * Restore a video mode
 */
void
TsengRestore(ScrnInfoPtr pScrn, vgaRegPtr vgaReg, TsengRegPtr tsengReg,
	     int flags)
{
    TsengPtr pTseng;
    unsigned char tmp;
    int iobase = VGAHWPTR(pScrn)->IOBase;

    PDEBUG("	TsengRestore\n");

    pTseng = TsengPTR(pScrn);

    vgaHWProtect(pScrn, TRUE);

    outb(0x3CD, 0x00);		       /* segment select bits 0..3 */
    outb(0x3CB, 0x00);	       /* segment select bits 4,5 */

    if (pTseng->ChipType == ET4000) {
        switch (pTseng->RAMDAC) {
        case STG1703:
#ifdef TODO
            /* Restore STG 170x GenDAC Command and PLL registers 
             * we share one data structure with the gendac code, so the names
             * are not too good.
             */
            
	    STG1703setIndex(0x24, tsengReg->pll.f2_M);
	    outb(0x3c6, tsengReg->pll.f2_N);	/* use autoincrement */

            STG1703setIndex(0x03, tsengReg->pll.ctrl);	/* primary pixel mode */
            outb(0x3c6, tsengReg->pll.ctrl);	/* secondary pixel mode */
            outb(0x3c6, tsengReg->pll.timingctrl);	/* pipeline timing control */
            usleep(500);		       /* 500 usec PLL settling time required */

            STG1703magic(0);

            outb(0x3C8, 0);
            (void)inb(0x3C6);
            (void)inb(0x3C6);
            (void)inb(0x3C6);
            (void)inb(0x3C6);
            outb(0x3C6, tsengReg->pll.cmd_reg); /* write enh command reg */
            outb(0x3C8, 0);
#else
             xf86DrvMsg(pScrn->scrnIndex, X_WARNING, "Not implemented!\n");
#endif
            break;
        case CH8398:
            outb(0x3C8, 0);
            (void)inb(0x3C6);
            (void)inb(0x3C6);
            (void)inb(0x3C6);
            (void)inb(0x3C6);
            outb(0x3C6, tsengReg->pll.cmd_reg); /* write enh command reg */
            
            inb(0x3c8);
            inb(0x3c6);
            inb(0x3c6);
            inb(0x3c6);
            inb(0x3c6);
            inb(0x3c6);
            outb(0x3c6, tsengReg->pll.timingctrl);

	    outb(iobase + 4, 0x31);
	    tmp = inb(iobase + 5);
	    outb(iobase + 5, tmp | (1 << 6));		/* Set RS2 through CS3 */
	    /* We are in ClockRAM mode 0x3c7 = CRA, 0x3c8 = CWA, 0x3c9 = CDR */
	    outb(0x3c7, tsengReg->pll.r_idx);
	    outb(0x3c8, 10);
	    outb(0x3c9, tsengReg->pll.f2_N);
	    outb(0x3c9, tsengReg->pll.f2_M);
	    outb(0x3c8, tsengReg->pll.w_idx);
	    usleep(500);
	    inb(0x3c7);		       /* reset sequence */
	    inb(0x3c8);		       /* loop to Clock Select Register */
	    inb(0x3c8);
	    inb(0x3c8);
	    inb(0x3c8);
	    outb(0x3c8, tsengReg->pll.ctrl);
	    outb(iobase + 4, 0x31);
	    outb(iobase + 5, (tmp & 0x3F));
            break;
        default:
            break;
	}
    } else {

	/* Restore ET6000 CLKDAC PLL registers */
	tmp = inb(pTseng->IOAddress + 0x67);	/* remember old CLKDAC index register pointer */
	outb(pTseng->IOAddress + 0x67, 2);
	outb(pTseng->IOAddress + 0x69, tsengReg->pll.f2_M);
	outb(pTseng->IOAddress + 0x69, tsengReg->pll.f2_N);
	/* set MClk values if needed, but don't touch them if not needed
         *
         * Since setting the MClk to highly illegal value results in a
         * total system crash, we'd better play it safe here.
         * N1 must be <= 4, and N2 should always be 1
         */
        if ((tsengReg->pll.MClkN & 0xf8) != 0x20) {
            xf86Msg(X_ERROR, "Internal Error in MClk registers: MClkM=0x%x, MClkN=0x%x\n",
		    tsengReg->pll.MClkM, tsengReg->pll.MClkN);
        } else {
            outb(pTseng->IOAddress + 0x67, 10);
            outb(pTseng->IOAddress + 0x69, tsengReg->pll.MClkM);
            outb(pTseng->IOAddress + 0x69, tsengReg->pll.MClkN);
	}
	/* restore old index register */
	outb(pTseng->IOAddress + 0x67, tmp);
    }

    if (pTseng->ChipType == ET6000) {
	outb(pTseng->IOAddress + 0x13, tsengReg->ET6K_13);
	outb(pTseng->IOAddress + 0x40, tsengReg->ET6K_40);
	outb(pTseng->IOAddress + 0x58, tsengReg->ET6K_58);
	outb(pTseng->IOAddress + 0x41, tsengReg->ET6K_41);
	outb(pTseng->IOAddress + 0x44, tsengReg->ET6K_44);
	outb(pTseng->IOAddress + 0x46, tsengReg->ET6K_46);
    }
    outw(iobase + 4, (tsengReg->CR3F << 8) | 0x3F);
    outw(iobase + 4, (tsengReg->CR30 << 8) | 0x30);
    outw(iobase + 4, (tsengReg->CR31 << 8) | 0x31);
    vgaHWRestore(pScrn, vgaReg, flags); /* TODO: does this belong HERE, in the middle? */
    outw(0x3C4, (tsengReg->SR06 << 8) | 0x06);
    outw(0x3C4, (tsengReg->SR07 << 8) | 0x07);
    tmp = inb(iobase + 0x0A);	       /* reset flip-flop */
    outb(0x3C0, 0x36);
    outb(0x3C0, tsengReg->ExtATC);
    outw(iobase + 4, (tsengReg->CR33 << 8) | 0x33);
    outw(iobase + 4, (tsengReg->CR34 << 8) | 0x34);
    outw(iobase + 4, (tsengReg->CR35 << 8) | 0x35);

    if (pTseng->ChipType == ET4000) {
	outw(iobase + 4, (tsengReg->CR37 << 8) | 0x37);
	outw(0x217a, (tsengReg->ExtIMACtrl << 8) | 0xF7);

	outw(iobase + 4, (tsengReg->CR32 << 8) | 0x32);
    }

    outb(0x3CD, tsengReg->ExtSegSel[0]);
    outb(0x3CB, tsengReg->ExtSegSel[1]);

    vgaHWProtect(pScrn, FALSE);

    /* 
     * We must change CRTC 0x36 only OUTSIDE the TsengProtect(pScrn,
     * TRUE)/TsengProtect(pScrn, FALSE) pair, because the sequencer reset
     * also resets the linear mode bits in CRTC 0x36.
     */
    if (pTseng->ChipType == ET4000)
	outw(iobase + 4, (tsengReg->CR36 << 8) | 0x36);
}

/*
 *
 */
Bool
TsengModeInit(ScrnInfoPtr pScrn, DisplayModePtr mode)
{
    vgaHWPtr hwp;
    TsengPtr pTseng = TsengPTR(pScrn);
    TsengRegPtr new = &(pTseng->ModeReg);
    TsengRegPtr initial = &(pTseng->SavedReg);
    int row_offset;
    int min_n2;
    int hdiv = 1, hmul = 1;

    PDEBUG("	TsengModeInit\n");

    switch (mode->PrivFlags) {
	case TSENG_MODE_PIXMUX:
	case TSENG_MODE_DACBUS16:
	    hdiv = pTseng->clockRange[1]->ClockDivFactor;
	    hmul = pTseng->clockRange[1]->ClockMulFactor;
	    break;
	default:
	    hdiv = pTseng->clockRange[0]->ClockDivFactor;
	    hmul = pTseng->clockRange[0]->ClockMulFactor;
    }

    /*
     * Modify mode timings accordingly
     */
    if (!mode->CrtcHAdjusted) {
	/* now divide and multiply the horizontal timing parameters as required */
	mode->CrtcHTotal = (mode->CrtcHTotal * hmul) / hdiv;
	mode->CrtcHDisplay = (mode->CrtcHDisplay * hmul) / hdiv;
	mode->CrtcHSyncStart = (mode->CrtcHSyncStart * hmul) / hdiv;
	mode->CrtcHSyncEnd = (mode->CrtcHSyncEnd * hmul) / hdiv;
	mode->CrtcHBlankStart = (mode->CrtcHBlankStart * hmul) / hdiv;
	mode->CrtcHBlankEnd = (mode->CrtcHBlankEnd * hmul) / hdiv;
	mode->CrtcHSkew = (mode->CrtcHSkew * hmul) / hdiv;
	if (pScrn->bitsPerPixel == 24) {
	    int rgb_skew;

	    /*
	     * in 24bpp, the position of the BLANK signal determines the
	     * phase of the R,G and B values. XFree86 sets blanking equal to
	     * the Sync, so setting the Sync correctly will also set the
	     * BLANK corectly, and thus also the RGB phase
	     */
	    rgb_skew = (mode->CrtcHTotal / 8 - mode->CrtcHBlankEnd / 8 - 1) % 3;
	    mode->CrtcHBlankEnd += rgb_skew * 8 + 24;
	    /* HBlankEnd must come BEFORE HTotal */
	    if (mode->CrtcHBlankEnd > mode->CrtcHTotal)
		mode->CrtcHBlankEnd -= 24;
	}
	mode->CrtcHAdjusted = TRUE;
    }

    /* set clockIndex to "2" for programmable clocks */
    mode->ClockIndex = 2;

    /* prepare standard VGA register contents */
    hwp = VGAHWPTR(pScrn);
    if (!vgaHWInit(pScrn, mode))
	return (FALSE);
    pScrn->vtSema = TRUE;

    /* prepare extended (Tseng) register contents */
    /* 
     * Start by copying all the saved registers in the "new" data, so we
     * only have to modify those that need to change.
     */

    memcpy(new, initial, sizeof(TsengRegRec));

    if (pScrn->bitsPerPixel < 8) {
	/* Don't ask me why this is needed on the ET6000 and not on the others */
	if (pTseng->ChipType == ET6000)
	    hwp->ModeReg.Sequencer[1] |= 0x04;
	row_offset = hwp->ModeReg.CRTC[19];
    } else {
	hwp->ModeReg.Attribute[16] = 0x01;	/* use the FAST 256 Color Mode */
	row_offset = pScrn->displayWidth >> 3;	/* overruled by 16/24/32 bpp code */
    }

    hwp->ModeReg.CRTC[20] = 0x60;
    hwp->ModeReg.CRTC[23] = 0xAB;
    new->SR06 = 0x00;
    new->SR07 = 0xBC;
    new->CR33 = 0x00;

    new->CR35 = (mode->Flags & V_INTERLACE ? 0x80 : 0x00)
	| 0x10
	| ((mode->CrtcVSyncStart & 0x400) >> 7)
	| (((mode->CrtcVDisplay - 1) & 0x400) >> 8)
	| (((mode->CrtcVTotal - 2) & 0x400) >> 9)
	| (((mode->CrtcVBlankStart - 1) & 0x400) >> 10);

    if (pScrn->bitsPerPixel < 8)
	new->ExtATC = 0x00;
    else
	new->ExtATC = 0x80;

    if (pScrn->bitsPerPixel >= 8) {
	if ((pTseng->ChipType == ET4000) && pTseng->FastDram) {
	    /*
	     *  make sure Trsp is no more than 75ns
	     *            Tcsw is 25ns
	     *            Tcsp is 25ns
	     *            Trcd is no more than 50ns
	     * Timings assume SCLK = 40MHz
	     *
	     * Note, this is experimental, but works for me (DHD)
	     */
	    /* Tcsw, Tcsp, Trsp */
	    new->CR32 &= ~0x1F;
	    if (initial->CR32 & 0x18)
		new->CR32 |= 0x08;
	    /* Trcd */
	    new->CR32 &= ~0x20;
	}
    }
    /*
     * Here we make sure that CRTC regs 0x34 and 0x37 are untouched, except for 
     * some bits we want to change. 
     * Notably bit 7 of CRTC 0x34, which changes RAS setup time from 4 to 0 ns 
     * (performance),
     * and bit 7 of CRTC 0x37, which changes the CRTC FIFO low treshold control.
     * At really high pixel clocks, this will avoid lots of garble on the screen 
     * when something is being drawn. This only happens WAY beyond 80 MHz 
     * (those 135 MHz ramdac's...)
     */
    if (pTseng->ChipType == ET4000) {
	if (!pTseng->SlowDram)
	    new->CR34 |= 0x80;
	if ((mode->Clock * pTseng->Bytesperpixel) > 80000)
	    new->CR37 |= 0x80;
	/*
	 * now on to the memory interleave setting (CR32 bit 7)
	 */
	if (pTseng->SetW32Interleave) {
	    if (pTseng->W32Interleave)
		new->CR32 |= 0x80;
	    else
		new->CR32 &= 0x7F;
	}

	/*
	 * CR34 bit 4 controls the PCI Burst option
	 */
	if (pTseng->SetPCIBurst) {
	    if (pTseng->PCIBurst)
		new->CR34 |= 0x10;
	    else
		new->CR34 &= 0xEF;
	}
    }

    /* prepare clock-related registers when not Legend.
     * cannot really SET the clock here yet, since the ET4000Save()
     * is called LATER, so it would save the wrong state...
     * ET4000Restore() is used to actually SET vga regs.
     */
    if (pTseng->ChipType == ET4000) {
        switch (pTseng->RAMDAC) {
        case STG1703:
            if (mode->PrivFlags == TSENG_MODE_PIXMUX)
                /* pixmux requires a post-div of 4 on ICS GenDAC clock generator */
                min_n2 = 2;
            else
                min_n2 = 0;
            TsengcommonCalcClock(mode->SynthClock, 1, 1, 31, min_n2, 3,
                                 100000, pTseng->max_vco_freq,
                                 &(new->pll.f2_M), &(new->pll.f2_N));
            
            new->pll.w_idx = 0;
            new->pll.r_idx = 0;
            
            break;
        case CH8398:
#ifdef TODO
            Chrontel8391CalcClock(mode->SynthClock, &temp1, &temp2, &temp3);
            new->pll.f2_N = (unsigned char)(temp2);
            new->pll.f2_M = (unsigned char)(temp1 | (temp3 << 6));
            /* ok LSB=f2_N and MSB=f2_M            */
            /* now set the Clock Select Register(CSR)      */
            new->pll.ctrl = (new->pll.ctrl | 0x90) & 0xF0;
            new->pll.timingctrl &= 0x1F;
            new->pll.r_idx = 0;
            new->pll.w_idx = 0;
#else
            xf86DrvMsg(pScrn->scrnIndex, X_WARNING, "Not implemented!\n");
#endif
            break;
        default:
            break;
        }
    } else {
	/* setting min_n2 to "1" will ensure a more stable clock ("0" is allowed though) */
	TsengcommonCalcClock(mode->SynthClock, 1, 1, 31, 1, 3, 100000,
	    pTseng->max_vco_freq,
	    &(new->pll.f2_M), &(new->pll.f2_N));
	/* above 130MB/sec, we enable the "LOW FIFO threshold" */
	if (mode->Clock * pTseng->Bytesperpixel > 130000) {
	    new->ET6K_41 |= 0x10;
	    if (pTseng->ChipRev == REV_ET6100)
		new->ET6K_46 |= 0x04;
	} else {
	    new->ET6K_41 &= ~0x10;
	    if (pTseng->ChipRev == REV_ET6100)
		new->ET6K_46 &= ~0x04;
	}

        /* according to Tseng Labs, N1 must be <= 4, and N2 should always be 1 for MClk */
        TsengcommonCalcClock(pTseng->MemClk, 1, 1, 4, 1, 1,
                             100000, pTseng->MaxClock * 2,
                             &(new->pll.MClkM), &(new->pll.MClkN));
	/* 
	 * Even when we don't allow setting the MClk value as described
	 * above, we can use the FAST/MED/SLOW DRAM options to set up
	 * the RAS/CAS delays as decided by the value of ET6K_44.
	 * This is also a more correct use of the flags, as it describes
	 * how fast the RAM works. [HNH].
	 */
	if (pTseng->FastDram)
	    new->ET6K_44 = 0x04; /* Fastest speed(?) */
	else if (pTseng->MedDram)
	    new->ET6K_44 = 0x15; /* Medium speed */
	else if (pTseng->SlowDram)
	    new->ET6K_44 = 0x35; /* Slow speed */
	else
	    ;		               /* keep current value */
    }
    /*
     * Set the clock selection bits. Because of the odd mapping between
     * Tseng clock select bits and what XFree86 does, "CSx" refers to a
     * register bit with the same name in the Tseng data books.
     *
     * XFree86 uses the following mapping:
     *
     *  Tseng register bit name		XFree86 clock select bit
     *	    CS0				    0
     *      CS1				    1
     *      CS2				    2
     *      MCLK/2			    3
     *      CS3				    4
     *      CS4				    not used
     */
    /* CS0 and CS1 are set by standard VGA code (vgaHW) */
    /* CS2 = CRTC 0x34 bit 1 */
    new->CR34 &= 0xFD;
    /* for programmable clocks: disable MCLK/2 and MCLK/4 independent of hibit */
    new->SR07 = (new->SR07 & 0xBE);
    /* clock select bit 4 = CS3 , clear CS4 */
    new->CR31 &= 0x3F;

    /*
     * linear mode handling
     */

    if (pTseng->ChipType == ET6000) {
        new->ET6K_13 = pTseng->FbAddress >> 24;
        new->ET6K_40 |= 0x09;
    } else {			       /* et4000 style linear memory */
        new->CR36 |= 0x10;
        new->CR30 = (pTseng->FbAddress >> 22) & 0xFF;
        hwp->ModeReg.Graphics[6] &= ~0x0C;
        new->ExtIMACtrl &= ~0x01;  /* disable IMA port (to get >1MB lin mem) */
    }

    /*
     * 16/24/32 bpp handling.
     */

    if (pScrn->bitsPerPixel >= 8) {
	tseng_set_ramdac_bpp(pScrn, mode);
	row_offset *= pTseng->Bytesperpixel;
    }
    /*
     * Horizontal overflow settings: for modes with > 2048 pixels per line
     */

    hwp->ModeReg.CRTC[19] = row_offset;
    new->CR3F = ((((mode->CrtcHTotal >> 3) - 5) & 0x100) >> 8)
	| ((((mode->CrtcHDisplay >> 3) - 1) & 0x100) >> 7)
	| ((((mode->CrtcHBlankStart >> 3) - 1) & 0x100) >> 6)
	| (((mode->CrtcHSyncStart >> 3) & 0x100) >> 4)
	| ((row_offset & 0x200) >> 3)
	| ((row_offset & 0x100) >> 1);

    /*
     * Enable memory mapped IO registers when acceleration is needed.
     */

    if (pTseng->UseAccel) {
	if (pTseng->ChipType == ET6000)
            new->ET6K_40 |= 0x02;	/* MMU can't be used here (causes system hang...) */
	else
	    new->CR36 |= 0x28;
    }
    vgaHWUnlock(hwp);		       /* TODO: is this needed (tsengEnterVT does this) */
    /* Program the registers */
    TsengRestore(pScrn, &hwp->ModeReg, new, VGA_SR_MODE);
    return TRUE;
}

/*
 * TsengCrtcDPMSSet --
 *
 * Sets VESA Display Power Management Signaling (DPMS) Mode.
 * This routine is for the ET4000W32P rev. c and later, which can
 * use CRTC indexed register 34 to turn off H/V Sync signals.
 *
 * '97 Harald Nordgård Hansen
 */
void
TsengCrtcDPMSSet(ScrnInfoPtr pScrn,
    int PowerManagementMode, int flags)
{
    unsigned char seq1, crtc34;
    int iobase = VGAHWPTR(pScrn)->IOBase;

    xf86EnableAccess(pScrn);
    switch (PowerManagementMode) {
    case DPMSModeOn:
    default:
	/* Screen: On; HSync: On, VSync: On */
	seq1 = 0x00;
	crtc34 = 0x00;
	break;
    case DPMSModeStandby:
	/* Screen: Off; HSync: Off, VSync: On */
	seq1 = 0x20;
	crtc34 = 0x01;
	break;
    case DPMSModeSuspend:
	/* Screen: Off; HSync: On, VSync: Off */
	seq1 = 0x20;
	crtc34 = 0x20;
	break;
    case DPMSModeOff:
	/* Screen: Off; HSync: Off, VSync: Off */
	seq1 = 0x20;
	crtc34 = 0x21;
	break;
    }
    outb(0x3C4, 0x01);		       /* Select SEQ1 */
    seq1 |= inb(0x3C5) & ~0x20;
    outb(0x3C5, seq1);
    outb(iobase + 4, 0x34);	       /* Select CRTC34 */
    crtc34 |= inb(iobase + 5) & ~0x21;
    outb(iobase + 5, crtc34);
}

/*
 * TsengHVSyncDPMSSet --
 *
 * Sets VESA Display Power Management Signaling (DPMS) Mode.
 * This routine is for Tseng et4000 chips that do not have any
 * registers to disable sync output.
 *
 * The "classic" (standard VGA compatible) method; disabling all syncs,
 * causes video memory corruption on Tseng cards, according to "Tseng
 * ET4000/W32 family tech note #20":
 *
 *   "Setting CRTC Indexed Register 17 bit 7 = 0 will disable the video
 *    syncs (=VESA DPMS power down), but will also disable DRAM refresh cycles"
 *
 * The method used here is derived from the same tech note, which describes
 * a method to disable specific sync signals on chips that do not have
 * direct support for it:
 *
 *    To get vsync off, program VSYNC_START > VTOTAL
 *    (approximately). In particular, the formula used is:
 *
 *        VSYNC.ADJ = (VTOT - VSYNC.NORM) + VTOT + 4
 *
 *        To test for this state, test if VTOT + 1 < VSYNC
 *
 *
 *    To get hsync off, program HSYNC_START > HTOTAL
 *    (approximately). In particular, the following formula is used:
 *
 *        HSYNC.ADJ = (HTOT - HSYNC.NORM) + HTOT + 7
 *
 *        To test for this state, test if HTOT + 3 < HSYNC
 *
 * The advantage of these formulas is that the ON state can be restored by
 * reversing the formula. The original state need not be stored anywhere...
 *
 * The trick in the above approach is obviously to put the start of the sync
 * _beyond_ the total H or V counter range, which causes the sync to never
 * toggle.
 */
void
TsengHVSyncDPMSSet(ScrnInfoPtr pScrn,
    int PowerManagementMode, int flags)
{
    unsigned char seq1, tmpb;
    unsigned int HSync, VSync, HTot, VTot, tmp;
    Bool chgHSync, chgVSync;
    int iobase = VGAHWPTR(pScrn)->IOBase;

    /* Code here to read the current values of HSync through VTot:
     *  HSYNC:
     *    bits 0..7 : CRTC index 0x04
     *    bit 8     : CRTC index 0x3F, bit 4
     */
    outb(iobase + 4, 0x04);
    HSync = inb(iobase + 5);
    outb(iobase + 4, 0x3F);
    HSync += (inb(iobase + 5) & 0x10) << 4;
    /*  VSYNC:
     *    bits 0..7 : CRTC index 0x10
     *    bits 8..9 : CRTC index 0x07 bits 2 (VSYNC bit 8) and 7 (VSYNC bit 9)
     *    bit 10    : CRTC index 0x35 bit 3
     */
    outb(iobase + 4, 0x10);
    VSync = inb(iobase + 5);
    outb(iobase + 4, 0x07);
    tmp = inb(iobase + 5);
    VSync += ((tmp & 0x04) << 6) + ((tmp & 0x80) << 2);
    outb(iobase + 4, 0x35);
    VSync += (inb(iobase + 5) & 0x08) << 7;
    /*  HTOT:
     *    bits 0..7 : CRTC index 0x00.
     *    bit 8     : CRTC index 0x3F, bit 0
     */
    outb(iobase + 4, 0x00);
    HTot = inb(iobase + 5);
    outb(iobase + 4, 0x3F);
    HTot += (inb(iobase + 5) & 0x01) << 8;
    /*  VTOT:
     *    bits 0..7 : CRTC index 0x06
     *    bits 8..9 : CRTC index 0x07 bits 0 (VTOT bit 8) and 5 (VTOT bit 9)
     *    bit 10    : CRTC index 0x35 bit 1
     */
    outb(iobase + 4, 0x06);
    VTot = inb(iobase + 5);
    outb(iobase + 4, 0x07);
    tmp = inb(iobase + 5);
    VTot += ((tmp & 0x01) << 8) + ((tmp & 0x20) << 4);
    outb(iobase + 4, 0x35);
    VTot += (inb(iobase + 5) & 0x02) << 9;

    /* Don't write these unless we have to. */
    chgHSync = chgVSync = FALSE;

    switch (PowerManagementMode) {
    case DPMSModeOn:
    default:
	/* Screen: On; HSync: On, VSync: On */
	seq1 = 0x00;
	if (HSync > HTot + 3) {	       /* Sync is off now, turn it on. */
	    HSync = (HTot - HSync) + HTot + 7;
	    chgHSync = TRUE;
	}
	if (VSync > VTot + 1) {	       /* Sync is off now, turn it on. */
	    VSync = (VTot - VSync) + VTot + 4;
	    chgVSync = TRUE;
	}
	break;
    case DPMSModeStandby:
	/* Screen: Off; HSync: Off, VSync: On */
	seq1 = 0x20;
	if (HSync <= HTot + 3) {       /* Sync is on now, turn it off. */
	    HSync = (HTot - HSync) + HTot + 7;
	    chgHSync = TRUE;
	}
	if (VSync > VTot + 1) {	       /* Sync is off now, turn it on. */
	    VSync = (VTot - VSync) + VTot + 4;
	    chgVSync = TRUE;
	}
	break;
    case DPMSModeSuspend:
	/* Screen: Off; HSync: On, VSync: Off */
	seq1 = 0x20;
	if (HSync > HTot + 3) {	       /* Sync is off now, turn it on. */
	    HSync = (HTot - HSync) + HTot + 7;
	    chgHSync = TRUE;
	}
	if (VSync <= VTot + 1) {       /* Sync is on now, turn it off. */
	    VSync = (VTot - VSync) + VTot + 4;
	    chgVSync = TRUE;
	}
	break;
    case DPMSModeOff:
	/* Screen: Off; HSync: Off, VSync: Off */
	seq1 = 0x20;
	if (HSync <= HTot + 3) {       /* Sync is on now, turn it off. */
	    HSync = (HTot - HSync) + HTot + 7;
	    chgHSync = TRUE;
	}
	if (VSync <= VTot + 1) {       /* Sync is on now, turn it off. */
	    VSync = (VTot - VSync) + VTot + 4;
	    chgVSync = TRUE;
	}
	break;
    }

    /* If the new hsync or vsync overflows, don't change anything. */
    if (HSync >= 1 << 9 || VSync >= 1 << 11) {
	ErrorF("tseng: warning: Cannot go into DPMS from this resolution.\n");
	chgVSync = chgHSync = FALSE;
    }
    /* The code to turn on and off video output is equal for all. */
    if (chgHSync || chgVSync) {
	outb(0x3C4, 0x01);	       /* Select SEQ1 */
	seq1 |= inb(0x3C5) & ~0x20;
	outb(0x3C5, seq1);
    }
    /* Then the code to write VSync and HSync to the card.
     *  HSYNC:
     *    bits 0..7 : CRTC index 0x04
     *    bit 8     : CRTC index 0x3F, bit 4
     */
    if (chgHSync) {
	outb(iobase + 4, 0x04);
	tmpb = HSync & 0xFF;
	outb(iobase + 5, tmpb);
	outb(iobase + 4, 0x3F);
	tmpb = (HSync & 0x100) >> 4;
	tmpb |= inb(iobase + 5) & ~0x10;
	outb(iobase + 5, tmpb);
    }
    /*  VSYNC:
     *    bits 0..7 : CRTC index 0x10
     *    bits 8..9 : CRTC index 0x07 bits 2 (VSYNC bit 8) and 7 (VSYNC bit 9)
     *    bit 10    : CRTC index 0x35 bit 3
     */
    if (chgVSync) {
	outb(iobase + 4, 0x10);
	tmpb = VSync & 0xFF;
	outb(iobase + 5, tmpb);
	outb(iobase + 4, 0x07);
	tmpb = (VSync & 0x100) >> 6;
	tmpb |= (VSync & 0x200) >> 2;
	tmpb |= inb(iobase + 5) & ~0x84;
	outb(iobase + 5, tmpb);
	outb(iobase + 4, 0x35);
	tmpb = (VSync & 0x400) >> 7;
	tmpb |= inb(iobase + 5) & ~0x08;
	outb(iobase + 5, tmpb);
    }
}
