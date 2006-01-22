/* $XFree86: xc/programs/Xserver/hw/xfree86/drivers/tseng/tseng_ramdac.c,v 1.27 2003/11/03 05:11:45 tsi Exp $ */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

/*
 *
 * Copyright 1993-1997 The XFree86 Project, Inc.
 *
 */

/*
 * tseng_ramdac.c.
 *
 * Much of this code was taken from the XF86_W32 (3.2) server [kmg]
 */

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
void
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
