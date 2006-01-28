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

static CARD8
vgaHWReadDacReadAddr(vgaHWPtr hwp)
{
    if (hwp->MMIOBase)
	return MMIO_IN8(hwp->MMIOBase, hwp->MMIOOffset + VGA_DAC_READ_ADDR);
    else
	return inb(hwp->PIOOffset + VGA_DAC_READ_ADDR);
}

#define VGA_BANK 0x3CB

void
vgaHWWriteBank(vgaHWPtr hwp, CARD8 value)
{
    if (hwp->MMIOBase)
	MMIO_OUT8(hwp->MMIOBase, hwp->MMIOOffset + VGA_BANK, value);
    else
	outb(hwp->PIOOffset + VGA_BANK, value);
}

CARD8
vgaHWReadBank(vgaHWPtr hwp)
{
    if (hwp->MMIOBase)
	return MMIO_IN8(hwp->MMIOBase, hwp->MMIOOffset + VGA_BANK);
    else
	return inb(hwp->PIOOffset + VGA_BANK);
}

#define VGA_SEGMENT 0x3CD

void
vgaHWWriteSegment(vgaHWPtr hwp, CARD8 value)
{
    if (hwp->MMIOBase)
	MMIO_OUT8(hwp->MMIOBase, hwp->MMIOOffset + VGA_SEGMENT, value);
    else
	outb(hwp->PIOOffset + VGA_SEGMENT, value);
}

CARD8
vgaHWReadSegment(vgaHWPtr hwp)
{
    if (hwp->MMIOBase)
	return MMIO_IN8(hwp->MMIOBase, hwp->MMIOOffset + VGA_SEGMENT);
    else
	return inb(hwp->PIOOffset + VGA_SEGMENT);
}

/*
 * 0x3D8 Tseng Display Mode Control
 */
#define VGA_MODE_CONTROL 0x08

void
vgaHWWriteModeControl(vgaHWPtr hwp, CARD8 value)
{
    if (hwp->MMIOBase)
        MMIO_OUT8(hwp->MMIOBase,
                  hwp->MMIOOffset + hwp->IOBase + VGA_MODE_CONTROL, value);
    else  
        outb(hwp->IOBase + hwp->PIOOffset + VGA_MODE_CONTROL, value);
}

/*
 * 0x3BF: Hercules compatibility mode. 
 * Enable/Disable Second page (B800h-BFFFh)
 */

#define VGA_HERCULES 0x3BF

void
vgaHWHerculesSecondPage(vgaHWPtr hwp, Bool Enable)
{
    CARD8 tmp;

    if (hwp->MMIOBase) {
        tmp = MMIO_IN8(hwp->MMIOBase, hwp->MMIOOffset + VGA_HERCULES);

        if (Enable)
            tmp |= 0x02;
        else
            tmp &= ~0x02;

        MMIO_OUT8(hwp->MMIOBase, hwp->MMIOOffset + VGA_HERCULES, tmp);
    } else {
        tmp = inb(hwp->PIOOffset + VGA_HERCULES);

        if (Enable)
            tmp |= 0x02;
        else
            tmp &= ~0x02;

        outb(hwp->PIOOffset + VGA_HERCULES, tmp);
    }
}

/*
 * ET6000 IO Range handling.
 *
 */
CARD8
ET6000IORead(TsengPtr pTseng, CARD8 Offset)
{
    return inb(pTseng->ET6000IOAddress + Offset);
}

void
ET6000IOWrite(TsengPtr pTseng, CARD8 Offset, CARD8 Value)
{
    outb(pTseng->ET6000IOAddress + Offset, Value);
}

/*
 * mode/clockRange->PrivFlags
 */
#define TSENG_MODE_NORMAL 0
#define TSENG_MODE_PIXMUX 1

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
        ET6000IORead(pTseng, 0x67);
        ET6000IOWrite(pTseng, 0x67, 0x0A);
        mclk = (ET6000IORead(pTseng, 0x69) + 2) * 14318;
        dbyte = ET6000IORead(pTseng, 0x69);
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
tseng_set_ramdac_bpp(ScrnInfoPtr pScrn, DisplayModePtr mode, TsengRegPtr Regs)
{
    TsengPtr pTseng = TsengPTR(pScrn);
    
    /* ATC index 0x16 -- bits-per-PCLK */
    Regs->ExtATC &= 0xCF;
    if (mode->PrivFlags == TSENG_MODE_PIXMUX)
        Regs->ExtATC |= 0x20;
    
    switch (pTseng->RAMDAC) {
    case STG1703:
        Regs->pll.cmd_reg &= 0x04; /* keep 7.5 IRE setup setting */
        Regs->pll.cmd_reg |= 0x18; /* enable ext regs and pixel modes */
        
        switch (pScrn->bitsPerPixel) {
        case 8:
            if (mode->PrivFlags == TSENG_MODE_PIXMUX)
                Regs->pll.ctrl = 0x05;
            else
                Regs->pll.ctrl = 0x00;
            break;
        case 15:
            if (mode->PrivFlags == TSENG_MODE_PIXMUX)
                Regs->pll.ctrl = 0x02;
            else
                Regs->pll.ctrl = 0x08;
            Regs->pll.cmd_reg |= 0xA0;
            break;
        /* !!!! We only should do pixmux from here on end !!!! */  
        case 16:
            Regs->pll.ctrl = 0x03;
            Regs->pll.cmd_reg |= 0xC0;
            break;
        case 24:
            Regs->pll.ctrl = 0x09;
            Regs->pll.cmd_reg |= 0xE0;
            break;
        case 32:
            Regs->pll.ctrl = 0x04;
            Regs->pll.cmd_reg |= 0xE0;
            break;
        default:
            Regs->pll.ctrl = 0;
            xf86DrvMsg(pScrn->scrnIndex, X_ERROR, "%s: STG1703 RAMDAC doesn't"
                       " support %dbpp.\n", __func__, pScrn->bitsPerPixel);
        }

        /* set PLL (input) range */
        if (mode->SynthClock <= 16000)
            Regs->pll.timingctrl = 0;
        else if (mode->SynthClock <= 32000)
            Regs->pll.timingctrl = 1;
        else if (mode->SynthClock <= 67500)
            Regs->pll.timingctrl = 2;
        else
            Regs->pll.timingctrl = 3;

        break;
    case CH8398:
        switch (pScrn->bitsPerPixel) {
        case 8:
            if (mode->PrivFlags == TSENG_MODE_PIXMUX)
                Regs->pll.cmd_reg = 0x24;
            else
                Regs->pll.cmd_reg = 0x04;
            break;
        case 15:
            if (mode->PrivFlags == TSENG_MODE_PIXMUX)
                Regs->pll.cmd_reg = 0x14;
            else
                Regs->pll.cmd_reg = 0xC4;
            break;
        case 16:
            if (mode->PrivFlags == TSENG_MODE_PIXMUX)
                Regs->pll.cmd_reg = 0x34;
            else
                Regs->pll.cmd_reg = 0x64;
            break;
        case 24:
            if (mode->PrivFlags == TSENG_MODE_PIXMUX)
                Regs->pll.cmd_reg = 0xB4;
            else
                Regs->pll.cmd_reg = 0x74;
            break;
        default:
            Regs->pll.cmd_reg = 0;
            xf86DrvMsg(pScrn->scrnIndex, X_ERROR, "%s: CH8398 RAMDAC doesn't"
                       " support %dbpp.\n", __func__, pScrn->bitsPerPixel);
        }
        break;
    default:
        return;
    }
}

#define MAX_TSENG_CLOCK 86000	       /* default max clock for standard boards */

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
void
tseng_clock_setup(ScrnInfoPtr pScrn)
{
    TsengPtr pTseng = TsengPTR(pScrn);
    int dacspeed, mem_bw;

    PDEBUG("	tseng_clock_setup\n");

    /* Set the min pixel clock */
    pTseng->MinClock = 12000; /* XXX Guess, need to check this */

    if (pTseng->ChipType == ET6000) {
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

        if (pTseng->ChipRev == REV_ET6100) {
            dacspeed = 175000;
            mem_bw = 280000; /* 275000 is _just_ not enough for 1152x864x24 @ 70Hz */
        } else { /* ET6000 */
            dacspeed = 135000;
            mem_bw = 225000;
        }

        pTseng->max_vco_freq = dacspeed*2+1;

        pTseng->clockRange[0] = xnfcalloc(sizeof(ClockRange), 1);
        pTseng->clockRange[0]->next = NULL;
        pTseng->clockRange[0]->minClock = pTseng->MinClock;
        pTseng->clockRange[0]->maxClock = 
            min(dacspeed, (mem_bw / pTseng->Bytesperpixel));
        pTseng->clockRange[0]->clockIndex = -1;      /* programmable -- not used */
        pTseng->clockRange[0]->interlaceAllowed = TRUE;
        pTseng->clockRange[0]->doubleScanAllowed = TRUE;
        pTseng->clockRange[0]->ClockMulFactor = 1;
        pTseng->clockRange[0]->ClockDivFactor = 1;
        pTseng->clockRange[0]->PrivFlags = TSENG_MODE_NORMAL;

    	pTseng->MaxClock = pTseng->clockRange[0]->maxClock;
    
    } else { /* ET4000 */
       if (pScrn->bitsPerPixel == 8)
            dacspeed = 135000; /* we can do PIXMUX */
        else
            dacspeed = MAX_TSENG_CLOCK;
        
        mem_bw	= 90000;
        if (pScrn->videoRam > 1024)
            mem_bw = 150000; /* interleaved DRAM gives 70% more bandwidth */ 

        pTseng->max_vco_freq = dacspeed*2+1;

        dacspeed = min(dacspeed, (mem_bw / pTseng->Bytesperpixel));

        /* 8bit dac bus */
        pTseng->clockRange[0] = xnfcalloc(sizeof(ClockRange), 1);
        pTseng->clockRange[0]->next = NULL;
        pTseng->clockRange[0]->minClock = pTseng->MinClock;
        pTseng->clockRange[0]->maxClock = 
            min(MAX_TSENG_CLOCK / pTseng->Bytesperpixel, dacspeed);
        pTseng->clockRange[0]->clockIndex = -1;      /* programmable -- not used */
        pTseng->clockRange[0]->interlaceAllowed = TRUE;
        pTseng->clockRange[0]->doubleScanAllowed = TRUE;
        pTseng->clockRange[0]->ClockMulFactor = pTseng->Bytesperpixel;
        pTseng->clockRange[0]->ClockDivFactor = 1;
        pTseng->clockRange[0]->PrivFlags = TSENG_MODE_NORMAL;

        /* 16bit dac bus */
        pTseng->clockRange[1] = xnfcalloc(sizeof(ClockRange), 1);
        pTseng->clockRange[0]->next = pTseng->clockRange[1];
        pTseng->clockRange[1]->next = NULL;

        if (pScrn->bitsPerPixel == 8) {
            pTseng->clockRange[1]->minClock = 75000;
            pTseng->clockRange[1]->maxClock = dacspeed;
        } else {
            pTseng->clockRange[1]->minClock = pTseng->MinClock;
            pTseng->clockRange[1]->maxClock = min((MAX_TSENG_CLOCK * 2) / pTseng->Bytesperpixel, dacspeed);
        }

        pTseng->clockRange[1]->clockIndex = -1; /* programmable -- not used */
        pTseng->clockRange[1]->interlaceAllowed = TRUE;
        pTseng->clockRange[1]->doubleScanAllowed = TRUE;
        pTseng->clockRange[1]->ClockMulFactor = pTseng->Bytesperpixel;
        pTseng->clockRange[1]->ClockDivFactor = 2;
        pTseng->clockRange[1]->PrivFlags = TSENG_MODE_PIXMUX;
        
        pTseng->MaxClock = pTseng->clockRange[1]->maxClock;
    }
}


/*
 *
 */
#define BASE_FREQ         14.31818     /* MHz */
static CARD16
ET6000CalcClock(long freq, int min_m, int min_n1, int max_n1, int min_n2,
                int max_n2, long freq_min, long freq_max)
{
    double ffreq, ffreq_min, ffreq_max;
    double div, diff, best_diff;
    unsigned int m;
    CARD8 n1, n2;
    CARD8 best_n1 = 16 + 2, best_n2 = 2, best_m = 125 + 2;
    CARD8 ndiv, mdiv;
    

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
	ndiv = (best_n1 - 2) | (best_n2 << 6);
    else
	ndiv = (best_n1 - 2) | (best_n2 << 5);
    mdiv = best_m - 2;

    return (ndiv << 8) | mdiv;
}

/*
 * adjust the current video frame (viewport) to display the mousecursor.
 */
void
TsengAdjustFrame(int scrnIndex, int x, int y, int flags)
{
    ScrnInfoPtr pScrn = xf86Screens[scrnIndex];
    TsengPtr pTseng = TsengPTR(pScrn);
    vgaHWPtr hwp = VGAHWPTR(pScrn);
    int Base;

    PDEBUG("	TsengAdjustFrame\n");

    if (pTseng->ShowCache && y)
        y += 256;

    if (pScrn->bitsPerPixel < 8)
	Base = (y * pScrn->displayWidth + x + 3) >> 3;
    else {
	Base = ((y * pScrn->displayWidth + x + 1) * pTseng->Bytesperpixel) >> 2;
	/* adjust Base address so it is a non-fractional multiple of pTseng->Bytesperpixel */
	Base -= (Base % pTseng->Bytesperpixel);
    }

    hwp->writeCrtc(hwp, 0x0C, (Base >> 8) & 0xFF);
    hwp->writeCrtc(hwp, 0x0D, Base & 0xFF);
    hwp->writeCrtc(hwp, 0x33, (Base >> 16) & 0x0F);
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
    vgaHWPtr hwp = VGAHWPTR(pScrn);
    TsengPtr pTseng = TsengPTR(pScrn);
    vgaRegPtr vgaReg;
    TsengRegPtr tsengReg;
    unsigned char temp, saveseg1 = 0, saveseg2 = 0;

    PDEBUG("	TsengSave\n");

    vgaReg = &hwp->SavedReg;
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
    temp = hwp->readCrtc(hwp, 0x34);
    tsengReg->CR34 = temp;

    if ((pTseng->ChipType == ET4000) &&
        ((pTseng->ChipRev == REV_A) || (pTseng->ChipRev == REV_B))) {
#ifdef OLD_CODE
	hwp->writeCrtc(hwp, 0x34, temp & 0x1F);
#else
	/* data books say translation ROM is controlled by bits 4 and 5 */
	hwp->writeCrtc(hwp, 0x34, temp & 0xCF);
#endif
    }

    saveseg1 = vgaHWReadSegment(hwp);
    vgaHWWriteSegment(hwp, 0x00); /* segment select 1 */

    saveseg2 = vgaHWReadBank(hwp);
    vgaHWWriteBank(hwp, 0x00); /* segment select 2 */

    tsengReg->ExtSegSel[0] = saveseg1;
    tsengReg->ExtSegSel[1] = saveseg2;

    tsengReg->CR33 = hwp->readCrtc(hwp, 0x33);
    tsengReg->CR35 = hwp->readCrtc(hwp, 0x35);

    if (pTseng->ChipType == ET4000) {
	tsengReg->CR36 = hwp->readCrtc(hwp, 0x36);
	tsengReg->CR37 = hwp->readCrtc(hwp, 0x37);
	tsengReg->CR32 = hwp->readCrtc(hwp, 0x32);
    }

    TsengCursorStore(pScrn, tsengReg);

    tsengReg->SR06 = hwp->readSeq(hwp, 0x06);
    tsengReg->SR07 = hwp->readSeq(hwp, 0x07) | 0x14;

    tsengReg->ExtATC = hwp->readAttr(hwp, 0x36);
    hwp->writeAttr(hwp, 0x36, tsengReg->ExtATC);

    if (pTseng->ChipType == ET4000) {
        switch (pTseng->RAMDAC) {
        case STG1703:
#ifdef TODO
            /* Save STG 1703 GenDAC Command and PLL registers 
             * unfortunately we reuse the gendac data structure, so the 
             * field names are not really good.
             */
            
            hwp->writeDacWriteAddr(hwp, 0x00);
            hwp->readDacMask(hwp);
            hwp->readDacMask(hwp);
            hwp->readDacMask(hwp);
            hwp->readDacMask(hwp);
            tsengReg->pll.cmd_reg = hwp->readDacMask(hwp); /* Enhanced command register */
            hwp->writeDacWriteAddr(hwp, 0x00);

            tsengReg->pll.f2_M = STG1703getIndex(0x24);	/* f2 PLL M divider */
            tsengReg->pll.f2_N = hwp->readDacMask(hwp);	/* f2 PLL N1/N2 divider */

            tsengReg->pll.ctrl = STG1703getIndex(0x03);	/* pixel mode select control */
            tsengReg->pll.timingctrl = STG1703getIndex(0x05);	/* pll timing control */
#else
             xf86DrvMsg(pScrn->scrnIndex, X_WARNING, "Not implemented!\n");
#endif
            break;
        case CH8398:
            hwp->writeDacWriteAddr(hwp, 0x00);
            hwp->readDacMask(hwp);
            hwp->readDacMask(hwp);
            hwp->readDacMask(hwp);
            hwp->readDacMask(hwp);
            tsengReg->pll.cmd_reg = hwp->readDacMask(hwp); /* Enhanced command register */
            hwp->writeDacWriteAddr(hwp, 0x00);

            vgaHWReadDacWriteAddr(hwp);
            hwp->readDacMask(hwp);
            hwp->readDacMask(hwp);
            hwp->readDacMask(hwp);
            hwp->readDacMask(hwp);
            hwp->readDacMask(hwp);
            tsengReg->pll.timingctrl = hwp->readDacMask(hwp);

            /* Save PLL */
            temp = hwp->readCrtc(hwp, 0x31);
            hwp->writeCrtc(hwp, 0x31, temp | (1 << 6));	/* set RS2 through CS3 */

            /* We are in ClockRAM mode 0x3c7 = CRA, 0x3c8 = CWA, 0x3c9 = CDR */
            tsengReg->pll.r_idx = vgaHWReadDacReadAddr(hwp);
            tsengReg->pll.w_idx = vgaHWReadDacWriteAddr(hwp);
            hwp->writeDacReadAddr(hwp, 10);
            tsengReg->pll.f2_N = hwp->readDacData(hwp);
            tsengReg->pll.f2_M = hwp->readDacData(hwp);
            hwp->writeDacReadAddr(hwp, tsengReg->pll.r_idx);
            /* !!! DAC_WRITE_ADDR is probably wrong here */
            vgaHWReadDacWriteAddr(hwp);  /* loop to Clock Select Register */
            vgaHWReadDacWriteAddr(hwp);
            vgaHWReadDacWriteAddr(hwp);
            vgaHWReadDacWriteAddr(hwp);
            tsengReg->pll.ctrl = vgaHWReadDacWriteAddr(hwp);

            hwp->writeCrtc(hwp, 0x31, temp);
            break;
        default:
            break;
	}
    } else {
	/* Save ET6000 CLKDAC PLL registers */
	temp = ET6000IORead(pTseng, 0x67); /* remember old CLKDAC index register pointer */

	ET6000IOWrite(pTseng, 0x67, 0x02);
	tsengReg->ET6K_PLL = ET6000IORead(pTseng, 0x69);
	tsengReg->ET6K_PLL |= ET6000IORead(pTseng, 0x69) << 8;

	/* save MClk values */
	ET6000IOWrite(pTseng, 0x67, 0x0A);
	tsengReg->ET6K_MClk = ET6000IORead(pTseng, 0x69);
	tsengReg->ET6K_MClk |= ET6000IORead(pTseng, 0x69) << 8;

	/* restore old index register */
	ET6000IOWrite(pTseng, 0x67, temp);
    }

    if (pTseng->ChipType == ET6000) {
	tsengReg->ET6K_13 = ET6000IORead(pTseng, 0x13);
	tsengReg->ET6K_40 = ET6000IORead(pTseng, 0x40);
	tsengReg->ET6K_58 = ET6000IORead(pTseng, 0x58);
	tsengReg->ET6K_41 = ET6000IORead(pTseng, 0x41);
	tsengReg->ET6K_44 = ET6000IORead(pTseng, 0x44);
	tsengReg->ET6K_46 = ET6000IORead(pTseng, 0x46);
    }

    tsengReg->CR30 = hwp->readCrtc(hwp, 0x30);
    tsengReg->CR31 = hwp->readCrtc(hwp, 0x31);
    tsengReg->CR3F = hwp->readCrtc(hwp, 0x3F);
}

/*
 * Restore a video mode
 */
void
TsengRestore(ScrnInfoPtr pScrn, vgaRegPtr vgaReg, TsengRegPtr tsengReg,
	     int flags)
{
    vgaHWPtr hwp = VGAHWPTR(pScrn);
    TsengPtr pTseng = TsengPTR(pScrn);
    CARD8 tmp;

    PDEBUG("	TsengRestore\n");

    vgaHWProtect(pScrn, TRUE);

    vgaHWWriteSegment(hwp, 0x00);		       /* segment select bits 0..3 */
    vgaHWWriteBank(hwp, 0x00); /* segment select bits 4,5 */

    if (pTseng->ChipType == ET4000) {
        switch (pTseng->RAMDAC) {
        case STG1703:
#ifdef TODO
            /* Restore STG 170x GenDAC Command and PLL registers 
             * we share one data structure with the gendac code, so the names
             * are not too good.
             */
            
	    STG1703setIndex(0x24, tsengReg->pll.f2_M);
	    hwp->writeDacMask(hwp, tsengReg->pll.f2_N); /* use autoincrement */

            STG1703setIndex(0x03, tsengReg->pll.ctrl); /* primary pixel mode */
            hwp->writeDacMask(hwp, tsengReg->pll.ctrl); /* secondary pixel mode */
            hwp->writeDacMask(hwp, tsengReg->pll.timingctrl); /* pipeline timing control */
            usleep(500); /* 500 usec PLL settling time required */

            STG1703magic(0);

            hwp->writeDacWriteAddr(hwp, 0x00);
            hwp->readDacMask(hwp);
            hwp->readDacMask(hwp);
            hwp->readDacMask(hwp);
            hwp->readDacMask(hwp);
            hwp->writeDacMask(hwp, tsengReg->pll.cmd_reg); /* write enh command reg */
            hwp->writeDacWriteAddr(hwp, 0x00);
#else
            xf86DrvMsg(pScrn->scrnIndex, X_WARNING, "Not implemented!\n");
#endif
            break;
        case CH8398:
            hwp->writeDacWriteAddr(hwp, 0x00);
            hwp->readDacMask(hwp);
            hwp->readDacMask(hwp);
            hwp->readDacMask(hwp);
            hwp->readDacMask(hwp);
            hwp->writeDacMask(hwp, tsengReg->pll.cmd_reg); /* write enh command reg */
            
            vgaHWReadDacWriteAddr(hwp);
            hwp->readDacMask(hwp);
            hwp->readDacMask(hwp);
            hwp->readDacMask(hwp);
            hwp->readDacMask(hwp);
            hwp->readDacMask(hwp);
            hwp->writeDacMask(hwp, tsengReg->pll.timingctrl);

	    tmp = hwp->readCrtc(hwp, 0x31);
	    hwp->writeCrtc(hwp, 0x31, tmp | (1 << 6)); /* Set RS2 through CS3 */

	    /* We are in ClockRAM mode 0x3c7 = CRA, 0x3c8 = CWA, 0x3c9 = CDR */
	    hwp->writeDacReadAddr(hwp, tsengReg->pll.r_idx);
	    hwp->writeDacWriteAddr(hwp, 0x0A);
	    hwp->writeDacData(hwp, tsengReg->pll.f2_N);
	    hwp->writeDacData(hwp, tsengReg->pll.f2_M);
	    hwp->writeDacWriteAddr(hwp, tsengReg->pll.w_idx);
	    usleep(500);
	    vgaHWReadDacReadAddr(hwp); /* reset sequence */
            /* !!! DAC_WRITE_ADDR is probably wrong here */
	    vgaHWReadDacWriteAddr(hwp); /* loop to Clock Select Register */
	    vgaHWReadDacWriteAddr(hwp);
	    vgaHWReadDacWriteAddr(hwp);
	    vgaHWReadDacWriteAddr(hwp);
	    hwp->writeDacWriteAddr(hwp, tsengReg->pll.ctrl);

            hwp->writeCrtc(hwp, 0x31, tmp & 0x3F);
            break;
        default:
            break;
	}
    } else {

	/* Restore ET6000 CLKDAC PLL registers */
	tmp = ET6000IORead(pTseng, 0x67);  /* remember old CLKDAC index register pointer */

	ET6000IOWrite(pTseng, 0x67, 0x02);
	ET6000IOWrite(pTseng, 0x69, tsengReg->ET6K_PLL & 0xFF);
	ET6000IOWrite(pTseng, 0x69, tsengReg->ET6K_PLL >> 8);

	/* set MClk values if needed, but don't touch them if not needed
         *
         * Since setting the MClk to highly illegal value results in a
         * total system crash, we'd better play it safe here.
         * N1 must be <= 4, and N2 should always be 1
         */
        if ((tsengReg->ET6K_MClk & 0xF800) != 0x2000) {
            xf86Msg(X_ERROR, "Internal Error in MClk registers: MClk: 0x%04X\n",
		    tsengReg->ET6K_MClk);
        } else {
            ET6000IOWrite(pTseng, 0x67, 10);
            ET6000IOWrite(pTseng, 0x69, tsengReg->ET6K_MClk & 0xFF);
            ET6000IOWrite(pTseng, 0x69, tsengReg->ET6K_MClk >> 8);
	}
	/* restore old index register */
        ET6000IOWrite(pTseng, 0x67, tmp);
    }

    if (pTseng->ChipType == ET6000) {
	ET6000IOWrite(pTseng, 0x13, tsengReg->ET6K_13);
	ET6000IOWrite(pTseng, 0x40, tsengReg->ET6K_40);
	ET6000IOWrite(pTseng, 0x58, tsengReg->ET6K_58);
	ET6000IOWrite(pTseng, 0x41, tsengReg->ET6K_41);
	ET6000IOWrite(pTseng, 0x44, tsengReg->ET6K_44);
	ET6000IOWrite(pTseng, 0x46, tsengReg->ET6K_46);
    }

    hwp->writeCrtc(hwp, 0x3F, tsengReg->CR3F);
    hwp->writeCrtc(hwp, 0x30, tsengReg->CR30);
    hwp->writeCrtc(hwp, 0x31, tsengReg->CR31);

    vgaHWRestore(pScrn, vgaReg, flags); /* TODO: does this belong HERE, in the middle? */

    hwp->writeSeq(hwp, 0x06, tsengReg->SR06);
    hwp->writeSeq(hwp, 0x07, tsengReg->SR07);

    hwp->writeAttr(hwp, 0x36, tsengReg->ExtATC);

    hwp->writeCrtc(hwp, 0x33, tsengReg->CR33);
    hwp->writeCrtc(hwp, 0x34, tsengReg->CR34);
    hwp->writeCrtc(hwp, 0x35, tsengReg->CR35);

    if (pTseng->ChipType == ET4000) {
        hwp->writeCrtc(hwp, 0x37, tsengReg->CR37);
	hwp->writeCrtc(hwp, 0x32, tsengReg->CR32);
    }

    TsengCursorRestore(pScrn, tsengReg);

    vgaHWWriteSegment(hwp, tsengReg->ExtSegSel[0]);
    vgaHWWriteBank(hwp, tsengReg->ExtSegSel[1]);

    vgaHWProtect(pScrn, FALSE);

    /* 
     * We must change CRTC 0x36 only OUTSIDE the TsengProtect(pScrn,
     * TRUE)/TsengProtect(pScrn, FALSE) pair, because the sequencer reset
     * also resets the linear mode bits in CRTC 0x36.
     */
    if (pTseng->ChipType == ET4000)
	hwp->writeCrtc(hwp, 0x36, tsengReg->CR36);
}

/*
 *
 */
Bool
TsengModeInit(ScrnInfoPtr pScrn, DisplayModePtr mode)
{
    vgaHWPtr hwp = VGAHWPTR(pScrn);
    TsengPtr pTseng = TsengPTR(pScrn);
    TsengRegPtr initial = &(pTseng->SavedReg);
    TsengRegRec new[1];
    int row_offset;

    PDEBUG("	TsengModeInit\n");

    if (pTseng->ChipType == ET4000) {
        int hmul = pTseng->Bytesperpixel, hdiv;

        if (mode->PrivFlags == TSENG_MODE_PIXMUX) {
            xf86DrvMsg(pScrn->scrnIndex, X_INFO,
                       "Setting up for 16bit DAC bus.\n");
            hdiv = 2;
        } else {
            xf86DrvMsg(pScrn->scrnIndex, X_INFO,
                       "Setting up for 8bit DAC bus.\n");
            hdiv = 1;
        }

        /* Modify mode timings accordingly
         * 
         * If we move the vgaHWInit code up here directly, we no longer have
         * to adjust mode->Crtc* but just handle things properly up here.
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
    }

    /* set clockIndex to "2" for programmable clocks */
    mode->ClockIndex = 2;

    /* prepare standard VGA register contents */
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
#ifdef TODO
            if ((pScrn->bitsPerPixel == 8) &&
                (mode->PrivFlags == TSENG_MODE_PIXMUX))
                /* pixmux requires a post-div of 4 on ICS GenDAC clock generator */
                min_n2 = 2;
            else
                min_n2 = 0;
            TsengcommonCalcClock(mode->SynthClock, 1, 1, 31, min_n2, 3,
                                 100000, pTseng->max_vco_freq,
                                 &(new->pll.f2_M), &(new->pll.f2_N));
            
            new->pll.w_idx = 0;
            new->pll.r_idx = 0;
#else
         xf86DrvMsg(pScrn->scrnIndex, X_WARNING, "Not implemented!\n");
#endif   
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
	new->ET6K_PLL = ET6000CalcClock(mode->SynthClock, 1, 1, 31, 1, 3,
                                        100000, pTseng->max_vco_freq);

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
        new->ET6K_MClk = ET6000CalcClock(pTseng->MemClk, 1, 1, 4, 1, 1,
                                         100000, pTseng->MaxClock * 2);

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
        new->CursorCtrl &= ~0x01;  /* disable IMA port (to get >1MB lin mem) */
    }

    /*
     * 16/24/32 bpp handling.
     */
    if (pScrn->bitsPerPixel >= 8) {
        if (pTseng->ChipType == ET6000) {
            /* ATC index 0x16 -- bits-per-PCLK */
            new->ExtATC &= 0xCF;
            new->ExtATC |= (pTseng->Bytesperpixel - 1) << 4;
            
            if (pScrn->bitsPerPixel == 15)
                new->ET6K_58 &= ~0x02; /* 5-5-5 RGB mode */
            else if (pScrn->bitsPerPixel == 16)
                new->ET6K_58 |= 0x02; /* 5-6-5 RGB mode */
        } else
            tseng_set_ramdac_bpp(pScrn, mode, new);
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
TsengCrtcDPMSSet(ScrnInfoPtr pScrn, int PowerManagementMode, int flags)
{
    vgaHWPtr hwp = VGAHWPTR(pScrn);
    CARD8 seq1, crtc34;

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

    seq1 |= hwp->readSeq(hwp, 0x01) & ~0x20;
    hwp->writeSeq(hwp, 0x01, seq1);

    crtc34 |= hwp->readCrtc(hwp, 0x34) & ~0x21;
    hwp->writeCrtc(hwp, 0x34, crtc34);
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
    vgaHWPtr hwp = VGAHWPTR(pScrn);
    CARD8 seq1, tmpb;
    CARD32 HSync, VSync, HTot, VTot, tmp;
    Bool chgHSync, chgVSync;

    /* Code here to read the current values of HSync through VTot:
     *  HSYNC:
     *    bits 0..7 : CRTC index 0x04
     *    bit 8     : CRTC index 0x3F, bit 4
     */
    HSync = hwp->readCrtc(hwp, 0x04);
    HSync += (hwp->readCrtc(hwp, 0x3F) & 0x10) << 4;

    /*  VSYNC:
     *    bits 0..7 : CRTC index 0x10
     *    bits 8..9 : CRTC index 0x07 bits 2 (VSYNC bit 8) and 7 (VSYNC bit 9)
     *    bit 10    : CRTC index 0x35 bit 3
     */
    VSync = hwp->readCrtc(hwp, 0x10);
    tmp = hwp->readCrtc(hwp, 0x07);
    VSync += ((tmp & 0x04) << 6) + ((tmp & 0x80) << 2);
    VSync += (hwp->readCrtc(hwp, 0x35) & 0x08) << 7;

    /*  HTOT:
     *    bits 0..7 : CRTC index 0x00.
     *    bit 8     : CRTC index 0x3F, bit 0
     */
    HTot = hwp->readCrtc(hwp, 0x00);
    HTot += (hwp->readCrtc(hwp, 0x3F) & 0x01) << 8;
    /*  VTOT:
     *    bits 0..7 : CRTC index 0x06
     *    bits 8..9 : CRTC index 0x07 bits 0 (VTOT bit 8) and 5 (VTOT bit 9)
     *    bit 10    : CRTC index 0x35 bit 1
     */
    VTot = hwp->readCrtc(hwp, 0x06);
    tmp = hwp->readCrtc(hwp, 0x07);
    VTot += ((tmp & 0x01) << 8) + ((tmp & 0x20) << 4);
    VTot += (hwp->readCrtc(hwp, 0x35) & 0x02) << 9;

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
	seq1 |= hwp->readSeq(hwp, 0x01) & ~0x20;
	hwp->writeSeq(hwp, 0x01, seq1);
    }
    /* Then the code to write VSync and HSync to the card.
     *  HSYNC:
     *    bits 0..7 : CRTC index 0x04
     *    bit 8     : CRTC index 0x3F, bit 4
     */
    if (chgHSync) {
	tmpb = HSync & 0xFF;
	hwp->writeCrtc(hwp, 0x04, tmpb);

	tmpb = (HSync & 0x100) >> 4;
	tmpb |= hwp->readCrtc(hwp, 0x3F) & ~0x10;
        hwp->writeCrtc(hwp, 0x3F, tmpb);
    }
    /*  VSYNC:
     *    bits 0..7 : CRTC index 0x10
     *    bits 8..9 : CRTC index 0x07 bits 2 (VSYNC bit 8) and 7 (VSYNC bit 9)
     *    bit 10    : CRTC index 0x35 bit 3
     */
    if (chgVSync) {
	tmpb = VSync & 0xFF;
	hwp->writeCrtc(hwp, 0x10, tmpb);

	tmpb = (VSync & 0x100) >> 6;
	tmpb |= (VSync & 0x200) >> 2;
	tmpb |= hwp->readCrtc(hwp, 0x07) & ~0x84;
	hwp->writeCrtc(hwp, 0x07, tmpb);

	tmpb = (VSync & 0x400) >> 7;
	tmpb |= hwp->readCrtc(hwp, 0x35) & ~0x08;
	hwp->writeCrtc(hwp, 0x35, tmpb);
    }
}
