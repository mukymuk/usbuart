#include "global.h"
#include "max32620.h"
#include "pwrman_regs.h"
#include "clkman_regs.h"
#include "usb_regs.h"

#include "usb.h"

void main(void)
{
    DBG(("test%d",3));
    usb_init();
    while( 1 );
}

