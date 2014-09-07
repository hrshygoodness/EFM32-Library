#include <stdio.h>
#include "lusb0_usb.h"

// Device vendor and product id.
#define MY_VID 0x2544
#define MY_PID 0x0007

// Device configuration and interface id.
#define MY_CONFIG 1
#define MY_INTF 0

// Device endpoint(s)
#define EP_IN  0x81
#define EP_OUT 0x01

#define BUF_SIZE 64

/* Buffer to receive incoming messages */
static char receiveBuffer[BUF_SIZE];

/* Data to send to device every 'tick' */
static char tickMessage[] = "tick";

/* Indicates whether a write (OUT transaction) is currently pending */
static bool pendingWrite = false;

/* Read and write contexts used by libusb */
void *async_read_context = NULL;
void *async_write_context = NULL;

/**********************************************************
 * Opens a device with the configured PID/VID.
 * Returns a device handle or NULL if no matching device
 * was found. 
 **********************************************************/
usb_dev_handle *open_dev(void)
{
    struct usb_bus *bus;
    struct usb_device *dev;

    for (bus = usb_get_busses(); bus; bus = bus->next)
    {
		printf("USB Bus: %s\n", bus->dirname);
        for (dev = bus->devices; dev; dev = dev->next)
        {
			printf("USB Device: 0x%.4x:0x%.4x\n", dev->descriptor.idVendor, dev->descriptor.idProduct);
            if (dev->descriptor.idVendor == MY_VID
                    && dev->descriptor.idProduct == MY_PID)
            {
				printf("Device found\n");
                return usb_open(dev);
            }
        }
    }
    return NULL;
}

/**********************************************************
 * Sends a 'tick' message to the device. This function
 * puts the message in a pending state and returns 
 * immediately.
 **********************************************************/
void sendTickMessage()
{
	int ret;

	/* Abort if previous message is still pending */
	if ( pendingWrite )
	{
		return;
	}

	/* Send the message */
	ret = usb_submit_async(async_write_context, tickMessage, strlen(tickMessage));
	if (ret < 0)
	{
		printf("Error submitting tick message: %s\n", usb_strerror());
	}
	else
	{
		/* Set the pending flag */
		pendingWrite = true;
	}
}

/**********************************************************
 * Checks if the last 'tick' message has been sent. 
 * Updates pending flag.
 **********************************************************/
void checkIfTicMessageIsSent(void)
{
	int ret;

	/* Check if tick message has been sent */
	if ( pendingWrite )
	{
		ret = usb_reap_async_nocancel(async_write_context, 0);
		if ( ret > 0 )
		{
			printf("Tick message sent\n");
			pendingWrite = false;
		}
	}
}

/**********************************************************
 * Checks if a message has been received. Prints the
 * message to stdout or simply returns if no message
 * has arrived.
 **********************************************************/
void receiveMessage()
{
	int ret;

	/* Check if message has been received */
	ret = usb_reap_async_nocancel(async_read_context, 0);
	if ( ret < 0 )
	{
	}
	else
	{
		printf("Received message: %s\n", receiveBuffer);

		/* Submit new read transfer to be ready for the next message */
		ret = usb_submit_async(async_read_context, receiveBuffer, sizeof(receiveBuffer));
		if (ret < 0)
		{
			printf("Error submitting read: %s\n", usb_strerror());
		}
	}
}


int main(void)
{
    usb_dev_handle *dev = NULL; /* the device handle */
    int ret;

	int i;
	

	/* Initialize the library */
    usb_init(); 

	/* Find all busses */    
    ret = usb_find_busses(); 

	/* Find all connected devices */
	usb_find_devices(); 

	/* Open device */
    if (!(dev = open_dev()))
    {
        printf("Error opening device: %s\n", usb_strerror());
        return 0;
    }
    else
    {
        printf("Opened device %.4x:%.4x opened\n", MY_VID, MY_PID);
    }

	/* Set configuration. This is needed before claiming an interface. */
    if (usb_set_configuration(dev, MY_CONFIG) < 0)
    {
        printf("Error setting configuration #%d: %s\n", MY_CONFIG, usb_strerror());
        usb_close(dev);
        return 0;
    }
    else
    {
        printf("Set configuration #%d\n", MY_CONFIG);
    }

	/* Claim interface. This is needed before communicating. */
    if (usb_claim_interface(dev, 0) < 0)
    {
        printf("Error claiming interface #%d:%s\n", MY_INTF, usb_strerror());
        usb_close(dev);
        return 0;
    }
    else
    {
        printf("Claimed interface #%d\n", MY_INTF);
    }

	/* Set up the write (OUT) transfer */
    ret = usb_bulk_setup_async(dev, &async_write_context, EP_OUT);
    if (ret < 0)
    {
        printf("Error setting up write transfer: %s\n", usb_strerror());
		return 0;
    }

	/* Set up the read (IN) transfer */
    ret = usb_bulk_setup_async(dev, &async_read_context, EP_IN);
    if (ret < 0)
    {
        printf("Error setting up read transfer: %s\n", usb_strerror());
		return 0;
    }


	/* Submit read transfer. This prepares to receive the transfer. */
	ret = usb_submit_async(async_read_context, receiveBuffer, sizeof(receiveBuffer));
	if (ret < 0)
	{
		printf("Error submitting read: %s\n", usb_strerror());
	}

	DWORD tickMs = GetTickCount();
	DWORD startTime = tickMs;

	i=0;
	while (1)
	{

		/* Time out after 10s */
		if ( GetTickCount() - startTime > 10000 )
		{
			break;
		}

		/* Send tick messages each second */
		if ( GetTickCount() - tickMs > 1000 )
		{
			sendTickMessage();

			tickMs += 1000;
		}

		/* Poll to see if any IN messages are pending */
		receiveMessage();

		/* Poll the OUT pipe to see if the message has been sent */
		checkIfTicMessageIsSent();
	
	}
	
	/* Free resources */
	usb_free_async(&async_read_context);
	usb_free_async(&async_write_context);
    usb_release_interface(dev, 0);

	/* Close connection */
    if (dev)
    {
        usb_close(dev);
    }

	printf("Done.\n");

    return 0;
}
