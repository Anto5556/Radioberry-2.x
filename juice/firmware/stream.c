
#include "stream.h"

FT_HANDLE ftHandleA;

int init_stream() {

	printf("Init device for iq stream handling...\r\n");

	FT_STATUS ftStatus = FT_OK;

	ftStatus = FT_OpenEx((void**)"radioberry-juice A", FT_OPEN_BY_DESCRIPTION, &ftHandleA);

	if (ftStatus == FT_OK)
	{ // Port opened successfully
		ftStatus |= FT_SetBitMode(ftHandleA, 0, 0);
		usleep(1000000);
		ftStatus |= FT_SetBitMode(ftHandleA, 0xFF, FT_BITMODE_SYNC_FIFO);
		ftStatus |= FT_SetLatencyTimer(ftHandleA, 1);
		//
		// Use 16 KiB USB transfer buffers. This kernel runs with 16 KiB pages,
		// and the D2XX driver silently drops whole stream frames with 64 KiB
		// buffers (URB misalignment/overrun on this page size).
		//
		ftStatus |= FT_SetUSBParameters(ftHandleA, 16384, 16384);
		ftStatus |= FT_SetFlowControl(ftHandleA, FT_FLOW_RTS_CTS, 0, 0);
		ftStatus |= FT_Purge(ftHandleA, FT_PURGE_RX | FT_PURGE_TX);
		ftStatus |= FT_SetTimeouts(ftHandleA, 1000, 1000);

		if (ftStatus != FT_OK) {
			FT_Close(ftHandleA);
			printf("Init device did not succeed for iq streaming.\r\n");
			return -1;
		}
	}
	else {
		printf("Opening FTDI device did not succeed.\r\n");
		return -1;
	}

	printf("Init device succeeded for iq streaming using FT245 protocol.\r\n");

	return 0;
};

int deinit_stream() {

	printf("Close device  for streaming.\r\n");
	FT_STATUS ftStatus = FT_OK;
	ftStatus |= FT_SetBitMode(ftHandleA, 0, 0);
	ftStatus |= FT_Purge(ftHandleA, FT_PURGE_RX | FT_PURGE_TX);

	if (FT_Close(ftHandleA) != FT_OK) return -1;

	return 0;
}


int write_stream(unsigned char stream[]){
	
	FT_STATUS ftStatus = FT_OK;

	DWORD EventDWord;
	DWORD RxBytes;
	DWORD TxBytes;
	DWORD numBytesSent;
	

	ftStatus = FT_Write(ftHandleA, stream, 1032, &numBytesSent);


	if (ftStatus == FT_OK)
	{
		// FT_Read OK
		if (1032!= numBytesSent) {
			fprintf(stderr, "ds stream time not complete; only %d bytes sent \n", numBytesSent);
			return -1;
		}
	}
	else
	{
		fprintf(stderr, "ds stream time out \n");
	}

	return 0;
	
}

#define RB_FRAME_SIZE 1032
#define RB_READ_CHUNK 8192

//
// The FPGA gateware produces a continuous stream of 1032-byte HPSDR
// protocol-1 frames, each starting with the header 0xEF 0xFE 0x01 0x06.
// The FT245 FIFO itself carries no framing, so a read that starts mid-frame
// (which happens whenever the FPGA has already filled its side of the FIFO
// before the client sends the Start command, or after a USB overrun has
// dropped bytes) stays misaligned forever: every forwarded chunk would then
// be rejected by the SDR client and the waterfall would freeze.
//
// read_stream() therefore buffers the raw bytes from the FTDI, re-synchronises
// on the frame header, and hands out only frame-aligned 1032-byte chunks.
// Partial reads (timeouts) are kept instead of discarded so that no stream
// bytes are lost.
//
// Reads fetch RB_READ_CHUNK bytes at a time (several frames' worth) so the
// D2XX driver moves data in bulk instead of being polled for every frame.
//
static unsigned char frame_buffer[RB_FRAME_SIZE * 2 + RB_READ_CHUNK];
static int            frame_len = 0;

int read_stream(unsigned char stream[]) {

	for (;;) {
		//
		// Do we have at least one full frame worth of bytes buffered?
		//
		if (frame_len >= RB_FRAME_SIZE) {
			//
			// Fast path: buffer starts with a frame header, so the whole
			// 1032-byte chunk is one aligned frame.
			//
			if (frame_buffer[0] == 0xEF && frame_buffer[1] == 0xFE &&
			    frame_buffer[2] == 0x01 && frame_buffer[3] == 0x06) {
				memcpy(stream, frame_buffer, RB_FRAME_SIZE);
				memmove(frame_buffer, frame_buffer + RB_FRAME_SIZE, frame_len - RB_FRAME_SIZE);
				frame_len -= RB_FRAME_SIZE;
				return 0;
			}
			//
			// Misaligned (stale FIFO content at stream start, or bytes lost
			// by a USB error). Search the buffer for the next frame header
			// and discard everything before it.
			//
			int hdr = -1;
			for (int i = 0; i + 4 <= frame_len; i++) {
				if (frame_buffer[i]     == 0xEF && frame_buffer[i + 1] == 0xFE &&
				    frame_buffer[i + 2] == 0x01 && frame_buffer[i + 3] == 0x06) {
					hdr = i;
					break;
				}
			}
			if (hdr < 0) {
				// No header in this batch; keep the last 3 bytes in case a
				// header straddles the end of the buffer, then fetch more.
				memmove(frame_buffer, frame_buffer + frame_len - 3, 3);
				frame_len = 3;
			} else {
				memmove(frame_buffer, frame_buffer + hdr, frame_len - hdr);
				frame_len -= hdr;
			}
			continue;
		}

		//
		// Fetch another chunk from the FTDI. Any bytes received are kept,
		// even if the read timed out before the full chunk arrived.
		//
		DWORD BytesReceived = 0;
		FT_STATUS ftStatus = FT_Read(ftHandleA, frame_buffer + frame_len,
		                             RB_READ_CHUNK, &BytesReceived);
		if (BytesReceived > 0) {
			frame_len += (int) BytesReceived;
			continue;
		}
		if (ftStatus != FT_OK) {
			fprintf(stderr, "us stream time out \n");
			return -2;
		}
		//
		// No bytes arrived on a successful read (the D2XX driver returns
		// immediately when its URB pipeline is busy). Do not spin: give the
		// FPGA time to deliver the next chunk, then keep buffering.
		//
		usleep(1000);
	}
}

// End of source.,
