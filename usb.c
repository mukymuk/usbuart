#include "global.h"
#include "usb.h"
#include "tmr_regs.h"

#include "usb_regs.h"
#include "max32620.h"

#include "pwrman_regs.h"
#include "clkman_regs.h"

//#undef DBG
//#define DBG(x)

#define ACM_SET_LINE_CODING         0x20
#define ACM_GET_LINE_CODING         0x21



#define USB_CONFIG_SELF_POWERED 0
#define ENDPOINT_COUNT 3

#define USB_BMATTRIBUTES_TRANSFER_TYPE_BULK         2
#define USB_BMATTRIBUTES_TRANSFER_TYPE_INTERRUPT    3

#define USB_BMATTRIBUTES_SYNC_TYPE_NONE             (0<<2)
#define USB_BMATTRIBUTES_SYNC_TYPE_ASYNC            (1<<2)
#define USB_BMATTRIBUTES_SYNC_TYPE_ADAPTIVE         (2<<2)
#define USB_BMATTRIBUTES_SYNC_TYPE_SYNC             (3<<2)

#define USB_BMATTRIBUTES_USAGE_TYPE_DATA            (0<<4)
#define USB_BMATTRIBUTES_USAGE_TYPE_FEEDBACK        (1<<4)
#define USB_BMATTRIBUTES_USAGE_TYPE_IMPLICIT_FEEDBACK (2<<4)

#define USB_DESCRIPTOR_TYPE_DEVICE          1
#define USB_DESCRIPTOR_TYPE_CONFIGURATION   2
#define USB_DESCRIPTOR_TYPE_STRING          3

#define USB_SETUP_REQUEST_TYPE_DIRECTION_HOST_TO_DEVICE 0
#define USB_SETUP_REQUEST_TYPE_DIRECTION_DEVICE_TO_HOST 1

#define USB_SETUP_REQUEST_TYPE_STANDARD   0
#define USB_SETUP_REQUEST_TYPE_CLASS      1
#define USB_SETUP_REQUEST_TYPE_VENDOR     2

#define USB_SETUP_REQUEST_TYPE_RECIPIENT_DEVICE       0
#define USB_SETUP_REQUEST_TYPE_RECIPIENT_INTERFACE    1
#define USB_SETUP_REQUEST_TYPE_RECIPIENT_ENDPOINT     2

#define USB_SETUP_REQUEST_GET_STATUS            0
#define USB_SETUP_REQUEST_CLEAR_FEATURE         1
#define USB_SETUP_REQUEST_SET_FEATURE           3
#define USB_SETUP_REQUEST_SET_ADDRESS           5
#define USB_SETUP_REQUEST_GET_DESCRIPTOR        6
#define USB_SETUP_REQUEST_SET_DESCRIPTOR        7
#define USB_SETUP_REQUEST_GET_CONFIGURATION     8
#define USB_SETUP_REQUEST_SET_CONFIGURATION     9
#define USB_SETUP_REQUEST_GET_INTERFACE         0x0A
#define USB_SETUP_REQUEST_SET_INTERFACE         0x11

#define DESC_DEVICE               1
#define DESC_CONFIG               2
#define DESC_STRING               3
#define DESC_INTERFACE            4
#define DESC_ENDPOINT             5
#define DESC_DEVICE_QUAL          6
#define DESC_OTHER_SPEED          7
#define DESC_IFACE_PWR            8

typedef struct
{
    uint8_t     recipient : 5;  // USB_SETUP_REQUEST_TYPE_RECIPIENT_*
    uint8_t     type      : 2;  // USB_SETUP_REQUEST_TYPE_*
    uint8_t     direction : 1;  // USB_SETUP_REQUEST_TYPE_DIRECTION_*
}
usb_setup_request_type_t;


typedef struct
{
    usb_setup_request_type_t        bmRequestType;  // USB_SETUP_REQUEST_TYPE_*
    uint8_t                         bRequest;       // USB_SETUP_REQUEST_*
    uint16_t                        wValue;
    uint16_t                        wIndex;
    uint16_t                        wLength;
}
usb_setup_t;

typedef union
{
    usb_setup_t setup;
    uint32_t    reg[2];
}
usb_setup_reg_t;

typedef struct
{
    uint8_t *           p_data;
    uint16_t            size;
    uint8_t             buffer[MXC_USB_MAX_PACKET];
}
endpoint_t;

static endpoint_t s_endpoint[ENDPOINT_COUNT];

#pragma pack(1)

typedef struct
{
    uint32_t  dwDTERate;
    uint8_t   bCharFormat;
    uint8_t   bParityType;
    uint8_t   bDataBits;
}
acm_line_coding_t;

typedef struct
{
    volatile uint32_t size;
    volatile uint32_t address;
}
usb_buffer_t;

typedef struct
{
    usb_buffer_t      buffer[2];
}
usb_endpoint_buffer;

typedef struct
{
    usb_endpoint_buffer      out;
    usb_endpoint_buffer      in;
}
usb_endpoint0_buffer;

typedef struct
{
    usb_endpoint0_buffer    endpoint0;
    usb_endpoint_buffer     endpoint[ENDPOINT_COUNT-1];
}
usb_sie_t;

typedef struct
{
    uint8_t      bLength;
    uint8_t      bDescriptorType;
}
usb_descriptor_header_t;

typedef struct
{
    usb_descriptor_header_t header;
    uint16_t bcdUSB;
    uint8_t bDeviceClass;
    uint8_t bDeviceSubClass;
    uint8_t bDeviceProtocol;
    uint8_t bMaxPacketSize;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t iManufacturer;
    uint8_t iProduct;
    uint8_t iSerialNumber;
    uint8_t bNumConfigurations;
}
usb_device_descriptor_t;

typedef struct
{
    usb_descriptor_header_t header;
    uint16_t wTotalLength;
    uint8_t bNumInterfaces;
    uint8_t bConfigurationValue;
    uint8_t iConfiguration;
    uint8_t bmAttributes;
    uint8_t bMaxPower;
}
usb_configuration_descriptor_header_t;

typedef struct
{
    usb_descriptor_header_t header;
    uint8_t bInterfaceNumber;
    uint8_t bAlternateSetting;
    uint8_t bNumEndpoints;
    uint8_t bInterfaceClass;
    uint8_t bInterfaceSubClass;
    uint8_t bInterfaceProtocol;
    uint8_t iInterface;
}
usb_interface_descriptor_t;

typedef struct
{
    usb_descriptor_header_t header;
    uint8_t                 bEndpointAddress;
    uint8_t                 bmAttributes;
    uint16_t                wMaxPacketSize;
    uint8_t                 bInterval;
}
usb_endpoint_descriptor_t;

typedef struct
{
    uint8_t bFunctionalLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubtype;
}
usb_class_descriptor_header_t;

typedef struct
{
    usb_class_descriptor_header_t   header;
    uint8_t                         bcdCDC[2];
}
usb_cdc_header_functional_t;

typedef struct
{
    usb_class_descriptor_header_t   header;
    uint8_t                         bmCapabilities;
    uint8_t                         bmDataInterface;
}
usb_cdc_call_management_t;

typedef struct
{
    usb_class_descriptor_header_t   header;
    uint8_t                         bmCapabilities;
}
usb_cdc_abstract_control_t;

typedef struct
{
    usb_class_descriptor_header_t   header;
    uint8_t                         bmMasterInterface;
    uint8_t                         bmSlaveInterface0;
}
usb_cdc_union_functional_t;

typedef struct
{
    usb_configuration_descriptor_header_t   header;
    usb_interface_descriptor_t              cdc_if;
    usb_cdc_header_functional_t             header_functional;
    usb_cdc_call_management_t               call_management;
    usb_cdc_abstract_control_t              abstract_control;
    usb_cdc_union_functional_t              union_functional;
    usb_endpoint_descriptor_t               ep_int_in;
    usb_interface_descriptor_t              data_if;
    usb_endpoint_descriptor_t               ep_bulk_out;
    usb_endpoint_descriptor_t               ep_bulk_in;
}
usb_configuration_descriptor_t;

typedef struct
{
    uint16_t    self_powered : 1;
    uint16_t    remote_wakeup : 1;
    uint16_t    reserved : 14;
}
get_device_status_t;

#define USB_ENDPOINT_CONTROL          0
#define USB_ENDPOINT_ACM_BULK_IN      2
#define USB_ENDPOINT_ACM_BULK_OUT     1
#define USB_ENDPOINT_CDC_INT_IN       3

#pragma pack(4)

typedef struct
{
    usb_device_descriptor_t         device;
    usb_configuration_descriptor_t  configuration;
}
descriptors_t;

#pragma pack()

#define CS_INTERFACE    0x24
#define CS_ENDPOINT     0x25

#define CDC_HEADER(length,subtype) { .bFunctionalLength = length, .bDescriptorType = CS_INTERFACE, .bDescriptorSubtype = subtype }
#define DESC_HEADER(length,type)    { .bLength = length, .bDescriptorType = type }



static const uint8_t s_lang_id_desc[4] =
{
    0x04,         /* bLength */
    0x03,         /* bDescriptorType */
    0x09, 0x04    /* bString = wLANGID (see usb_20.pdf 9.6.7 String) */
};

static const uint8_t s_mfg_id_desc[34] =
{
    34, 0x03,
    'M', 0,
    'a', 0,
    'x', 0,
    'i', 0,
    'm', 0,
    ' ', 0,
    'I', 0,
    'n', 0,
    't', 0,
    'e', 0,
    'g', 0,
    'r', 0,
    'a', 0,
    't', 0,
    'e', 0,
    'd', 0,
};

static const uint8_t s_prod_id_desc[34] =
{
    34, 0x03,
    'M', 0,
    'A', 0,
    'X', 0,
    '3', 0,
    '2', 0,
    '6', 0,
    '2', 0,
    '0', 0,
    ' ', 0,
    'C', 0,
    'D', 0,
    'C', 0,
    '-', 0,
    'A', 0,
    'C', 0,
    'M', 0,
};

static const uint8_t * s_strings[3] =
{
    s_lang_id_desc,
    s_mfg_id_desc,
    s_prod_id_desc
};

static const descriptors_t s_descriptors =
{
    .device =
    {
        .header = DESC_HEADER( sizeof(s_descriptors.device), DESC_DEVICE ),
        .bcdUSB = 0x0110,
        .bDeviceClass = 0x02,
        .bDeviceSubClass = 0,
        .bDeviceProtocol = 0,
        .bMaxPacketSize = MXC_USB_MAX_PACKET,
        .idVendor = 0x0B6A,
        .idProduct = 0x003C,
        .iManufacturer = 1,
        .iProduct = 2,
        .iSerialNumber = 0,
        .bNumConfigurations = 1
    },
    .configuration =
    {
        .header =
        {
            .header = DESC_HEADER( sizeof(s_descriptors.configuration.header), DESC_CONFIG ),
            .wTotalLength = sizeof(s_descriptors.configuration),
            .bNumInterfaces = 2,
            .bConfigurationValue = 1,
            .iConfiguration = 0,
            .bmAttributes = 0x80, // bus powered
            .bMaxPower = 50 // 100mA
        },
        .cdc_if =
        {
            .header = DESC_HEADER( sizeof(s_descriptors.configuration.cdc_if), DESC_INTERFACE ),
            .bInterfaceNumber = 0,
            .bAlternateSetting = 0,
            .bNumEndpoints = 1,
            .bInterfaceClass = 2,
            .bInterfaceSubClass = 2,
            .bInterfaceProtocol = 1,
            .iInterface = 0
        },
        .header_functional =
        {
            .header = CDC_HEADER(sizeof(s_descriptors.configuration.header_functional),0),
            .bcdCDC = { 0x10, 0x01 }
        },
        .call_management =
        {
            .header = CDC_HEADER(sizeof(s_descriptors.configuration.call_management),1),
            .bmCapabilities = 0,
            .bmDataInterface = 1
        },
        .abstract_control =
        {
            .header = CDC_HEADER(sizeof(s_descriptors.configuration.abstract_control),2),
            .bmCapabilities = 0
        },
        .union_functional =
        {
            .header = CDC_HEADER(sizeof(s_descriptors.configuration.union_functional),6),
            .bmMasterInterface = 0,
            .bmSlaveInterface0 = 1
        },
        .ep_int_in =
        {
            .header = DESC_HEADER(sizeof(s_descriptors.configuration.ep_int_in),DESC_ENDPOINT),
            .bEndpointAddress = 0x80 | USB_ENDPOINT_CDC_INT_IN,
            .bmAttributes = USB_BMATTRIBUTES_TRANSFER_TYPE_INTERRUPT|USB_BMATTRIBUTES_SYNC_TYPE_NONE|USB_BMATTRIBUTES_USAGE_TYPE_DATA,
            .wMaxPacketSize = MXC_USB_MAX_PACKET,
            .bInterval = 0xFF
        },
        .data_if =
        {
            .header = DESC_HEADER( sizeof(s_descriptors.configuration.data_if), DESC_INTERFACE ),
            .bInterfaceNumber = 1,
            .bAlternateSetting = 0,
            .bNumEndpoints = 2,
            .bInterfaceClass = 10,
            .bInterfaceSubClass = 0,
            .bInterfaceProtocol = 0,
            .iInterface = 0
        },
        .ep_bulk_out =
        {
            .header = DESC_HEADER(sizeof(s_descriptors.configuration.ep_bulk_out),DESC_ENDPOINT),
            .bEndpointAddress = USB_ENDPOINT_ACM_BULK_OUT,
            .bmAttributes = USB_BMATTRIBUTES_TRANSFER_TYPE_BULK|USB_BMATTRIBUTES_SYNC_TYPE_NONE|USB_BMATTRIBUTES_USAGE_TYPE_DATA,
            .wMaxPacketSize = MXC_USB_MAX_PACKET,
            .bInterval = 0
        },
        .ep_bulk_in =
        {
            .header = DESC_HEADER(sizeof(s_descriptors.configuration.ep_bulk_in),DESC_ENDPOINT),
            .bEndpointAddress = 0x80 | USB_ENDPOINT_ACM_BULK_IN,
            .bmAttributes = USB_BMATTRIBUTES_TRANSFER_TYPE_BULK|USB_BMATTRIBUTES_SYNC_TYPE_NONE|USB_BMATTRIBUTES_USAGE_TYPE_DATA,
            .wMaxPacketSize = MXC_USB_MAX_PACKET,
            .bInterval = 0
        },
    }
};

static volatile __attribute__ ((aligned (512))) usb_sie_t s_usb_sie;

static void inline usb_stall( uint8_t ep )
{
    MXC_USB->ep[ep] |= MXC_F_USB_EP_STALL | MXC_F_USB_EP_ST_STALL;
}


void usb_init( void )
{
    uint8_t i;

    MXC_TMR0->term_cnt32 = 0xFFFFFFF;

    MXC_TMR0->ctrl = MXC_V_TMR_CTRL_MODE_CONTINUOUS << MXC_F_TMR_CTRL_MODE_POS | MXC_F_TMR_CTRL_ENABLE0;

    MXC_CLKMAN->clk_ctrl = MXC_F_CLKMAN_CLK_CTRL_USB_CLOCK_ENABLE;


    MXC_PWRMAN->pwr_rst_ctrl |= MXC_F_PWRMAN_PWR_RST_CTRL_USB_POWERED;

    for(i=0;i<ENDPOINT_COUNT;i++)
    {
        s_usb_sie.endpoint[i].buffer[0].address = (int32_t)s_endpoint[i+1].buffer;
        s_usb_sie.endpoint[i].buffer[0].size = MXC_USB_MAX_PACKET;
    }
    s_usb_sie.endpoint0.in.buffer[0].address = (int32_t)s_endpoint[0].buffer;
    s_usb_sie.endpoint0.out.buffer[0].address = (int32_t)s_endpoint[0].buffer;
    MXC_USB->ep[USB_ENDPOINT_CONTROL] = MXC_F_USB_EP_INT_EN | MXC_V_USB_EP_DIR_CONTROL << MXC_F_USB_EP_DIR_POS;
    MXC_USB->ep[USB_ENDPOINT_ACM_BULK_IN] = MXC_F_USB_EP_INT_EN | MXC_V_USB_EP_DIR_IN << MXC_F_USB_EP_DIR_POS;
    MXC_USB->ep[USB_ENDPOINT_ACM_BULK_OUT] = MXC_F_USB_EP_INT_EN | MXC_V_USB_EP_DIR_OUT << MXC_F_USB_EP_DIR_POS;
    //MXC_USB->ep[USB_ENDPOINT_CDC_INT_IN] = MXC_F_USB_EP_INT_EN | MXC_V_USB_EP_DIR_IN << MXC_F_USB_EP_DIR_POS;

    MXC_USB->cn = 1;
    MXC_USB->ep_base = (uint32_t)&s_usb_sie;
    MXC_USB->dev_inten = MXC_F_USB_DEV_INTEN_SETUP | MXC_F_USB_DEV_INTEN_EP_IN | MXC_F_USB_DEV_INTEN_EP_OUT;
    MXC_USB->dev_cn = (MXC_F_USB_DEV_CN_CONNECT | MXC_F_USB_DEV_CN_FIFO_MODE);

    MXC_USB->out_owner = (1<<USB_ENDPOINT_ACM_BULK_OUT);
    usb_stall(USB_ENDPOINT_ACM_BULK_IN);
    usb_stall(USB_ENDPOINT_CDC_INT_IN);
    NVIC_EnableIRQ(USB_IRQn);
}

static void inline usb_ack( uint8_t ep )
{
    MXC_USB->ep[ep] |= MXC_F_USB_EP_ST_ACK;
}

static void read_continue( uint8_t ep )
{
    endpoint_t *p_ep = &s_endpoint[ep];
    uint32_t size = p_ep->size;
    if( ep )
    {
        size = s_usb_sie.endpoint[ep-1].buffer[0].size;
        s_usb_sie.endpoint[ep-1].buffer[0].size = MXC_USB_MAX_PACKET;
    }
    else
    {
        size = s_usb_sie.endpoint0.out.buffer[0].size;
        s_usb_sie.endpoint0.out.buffer[0].size = MXC_USB_MAX_PACKET;
    }

    if( size >  p_ep->size )
        size =  p_ep->size;

    memcpy( p_ep->p_data, p_ep->buffer, size );

    p_ep->size -= size;
    p_ep->p_data += size;

    if( !p_ep->size )
    {
        usb_ack(ep);
    }

    MXC_USB->out_owner = (1<<ep);
}

static void usb_read( uint8_t ep, void *pv_data, uint8_t size )
{
    endpoint_t *p_ep = &s_endpoint[ep];
    p_ep->size = size;
    p_ep->p_data = pv_data;
    if( ep )
        s_usb_sie.endpoint[ep-1].buffer[0].size = MXC_USB_MAX_PACKET;
    else
        s_usb_sie.endpoint0.out.buffer[0].size = MXC_USB_MAX_PACKET;
    MXC_USB->out_owner = (1<<ep);

}

static void write_continue( uint8_t ep )
{
    endpoint_t *p_ep = &s_endpoint[ep];
    uint32_t size = p_ep->size;
    if( size )
    {
        if( size > MXC_USB_MAX_PACKET )
            size = MXC_USB_MAX_PACKET;

        memcpy( p_ep->buffer, p_ep->p_data, size );
        if( ep )
            s_usb_sie.endpoint[ep-1].buffer[0].size = size;
        else
            s_usb_sie.endpoint0.in.buffer[0].size = size;

        MXC_USB->in_owner = (1<<ep);

        p_ep->size -= size;
        p_ep->p_data += size;
    }
    else
    {
        usb_ack(ep);
    }
}

static void usb_write( uint8_t ep, const void *pv_data, uint8_t size )
{
    endpoint_t *p_ep = &s_endpoint[ep];

    p_ep->size = size;
    p_ep->p_data = (void*)pv_data;
    write_continue(ep);
}

static void setup_standard_device_request( const usb_setup_t * p )
{
    switch( p->bRequest )
    {
        case USB_SETUP_REQUEST_GET_DESCRIPTOR:
        {
            uint8_t type = p->wValue >> 8;
            switch( type )
            {
                case USB_DESCRIPTOR_TYPE_DEVICE:
                {
                    DBG(("USB_DESCRIPTOR_TYPE_DEVICE\n"));
                    usb_write( USB_ENDPOINT_CONTROL, &s_descriptors.device, MIN(sizeof(s_descriptors.device),p->wLength) );
                    return;
                }
                case USB_DESCRIPTOR_TYPE_CONFIGURATION:
                {
                    DBG(("USB_DESCRIPTOR_TYPE_CONFIGURATION\n"));
                    usb_write( USB_ENDPOINT_CONTROL, &s_descriptors.configuration, MIN(sizeof(s_descriptors.configuration),p->wLength) );
                    return;
                }
                case USB_DESCRIPTOR_TYPE_STRING:
                {
                    DBG(("USB_DESCRIPTOR_TYPE_STRING\n"));
                    uint8_t ndx = p->wValue & 0xFF;
                    if( ndx < ARRAY_COUNT(s_strings) )
                    {
                        usb_write(USB_ENDPOINT_CONTROL, s_strings[ndx], MIN(s_strings[ndx][0],p->wLength) );
                        return;
                    }
                    break;
                }
            }
            break;
        }
        case USB_SETUP_REQUEST_SET_CONFIGURATION:
        {
            usb_ack(USB_ENDPOINT_CONTROL);
            return;
        }
    }
    usb_stall(USB_ENDPOINT_CONTROL);
}

static void setup_standard_interface_request( const usb_setup_t * p )
{
    switch( p->bRequest )
    {
        case USB_SETUP_REQUEST_GET_INTERFACE:
        {
            uint8_t zero = 0;
            DBG(("USB_SETUP_REQUEST_GET_INTERFACE\n"));
            usb_write( USB_ENDPOINT_CONTROL, &zero, sizeof(zero) );
            return;
        }
        case USB_SETUP_REQUEST_SET_INTERFACE:
        {
            DBG(("USB_SETUP_REQUEST_SET_INTERFACE\n"));
            usb_ack(USB_ENDPOINT_CONTROL);
            return;
        }
    }
    usb_stall(USB_ENDPOINT_CONTROL);
}

static void setup_class_interface_request( const usb_setup_t * p )
{
    static acm_line_coding_t line_coding =
    {
        .dwDTERate = 115200,
        .bDataBits = 8,
        .bParityType = 0,
        .bCharFormat = 0
    };

    switch( p->bRequest )
    {
        case ACM_SET_LINE_CODING:
        {
            DBG(("ACM_SET_LINE_CODING\n"));
            usb_read( USB_ENDPOINT_CONTROL, &line_coding, sizeof(line_coding) );
            return;
        }
        case ACM_GET_LINE_CODING:
        {
            DBG(("ACM_GET_LINE_CODING\n"));
            usb_write( USB_ENDPOINT_CONTROL, &line_coding, sizeof(line_coding) );
            return;
        }
    }
    usb_stall(USB_ENDPOINT_CONTROL);
}

static void setup( void )
{
    usb_setup_reg_t setup;
    const usb_setup_t *p;
    {
        // copy setup payload into RAM so as to avoid non 32-bit access bus faults
        const usb_setup_reg_t *p_reg = (const usb_setup_reg_t*)&MXC_USB->setup0;
        setup.reg[0] = p_reg->reg[0];
        setup.reg[1] = p_reg->reg[1];
        p = &setup.setup;
    }
    switch( p->bmRequestType.type )
    {
        case USB_SETUP_REQUEST_TYPE_STANDARD:
        {
            switch( p->bmRequestType.recipient )
            {
                case USB_SETUP_REQUEST_TYPE_RECIPIENT_DEVICE:
                {
                    DBG(("USB_SETUP_REQUEST_TYPE_RECIPIENT_DEVICE\n"));
                    setup_standard_device_request(p);
                    return;
                }
                case USB_SETUP_REQUEST_TYPE_RECIPIENT_INTERFACE:
                {
                    DBG(("USB_SETUP_REQUEST_TYPE_RECIPIENT_INTERFACE\n"));
                    setup_standard_interface_request(p);
                    return;
                }
            }
            break;
        }
        case USB_SETUP_REQUEST_TYPE_CLASS:
        {
            DBG( ("USB_SETUP_REQUEST_TYPE_CLASS\n"));
            setup_class_interface_request(p);
            return;
        }
    }
    usb_stall(USB_ENDPOINT_CONTROL);
}


void USB_IRQHandler( void )
{
    uint32_t intfl = MXC_USB->dev_intfl & MXC_USB->dev_inten;
    if( intfl & MXC_F_USB_DEV_INTFL_EP_IN )
    {
        uint32_t in_int = MXC_USB->in_int;
        for(int8_t ep=0;ep<ENDPOINT_COUNT;ep++)
        {
           if( in_int & 1<<ep )
           {
               DBG(("MXC_F_USB_DEV_INTFL_EP_IN = %d\n", ep));
               write_continue( ep );
           }
        }
        MXC_USB->in_int = in_int;
    }
    if( intfl & MXC_F_USB_DEV_INTFL_EP_OUT )
    {
        
        uint32_t out_int = MXC_USB->out_int;
        for(int8_t ep=0;ep<ENDPOINT_COUNT;ep++)
        {
            if( out_int & 1<<ep )
            {
                DBG(("MXC_F_USB_DEV_INTFL_EP_OUT = %d\n", ep));
                read_continue( ep );
            }
        }
        MXC_USB->out_int = out_int;
    }
    if( intfl & MXC_F_USB_DEV_INTFL_SETUP )
    {
        DBG(("MXC_F_USB_DEV_INTFL_SETUP\n"));
        setup();
    }
    MXC_USB->dev_intfl = intfl;
}
