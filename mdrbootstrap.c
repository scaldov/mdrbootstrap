#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <termios.h>
#include <pthread.h>
#include <string.h>
#include <signal.h>
#include <errno.h>
#include <ctype.h>

#ifdef __CYGWIN__
#include <windows.h>
#include <io.h>
#include <sys/socket.h> // cygwin defines FIONREAD in  socket.h  instead of  ioctl.h
#endif


/* Macros for converting between hex and binary. */
#define NIBBLE(x)	   (isdigit(x) ? (x)-'0' : tolower(x)+10-'a')
#define HEX(buffer)	 ((NIBBLE((buffer)[0])<<4) + NIBBLE((buffer)[1]))

int rtty_set(int fd, int bps);
int tty_select(int fd, unsigned int us);
int tty_poll(int dev);

unsigned fw_base;
int fw_len;
unsigned char *fw_data;
unsigned boot_base;
int boot_len;
unsigned char *boot_data;

int tty_select(int fd, unsigned int us)
{
#ifndef __CYGWIN__
	fd_set fds_read;
	struct timeval tv;
	int r;
	FD_ZERO(&fds_read);
	FD_SET(fd, &fds_read);
	tv.tv_sec = us/1000000;
	tv.tv_usec = us - tv.tv_sec*1000000;
	r = select(fd+1, &fds_read, 0, 0, &tv);
	if(r>=0) {
		if(FD_ISSET(fd, &fds_read)) return 1;
		else return 0;
	} else {
		return (errno == EBADF)? -1 : 0; 
	}
#else
	HANDLE hComm;
	COMSTAT comm_stat;
	DWORD comm_errors;
	LARGE_INTEGER frequency;        // ticks per second
	LARGE_INTEGER t1, t2;           // ticks

	//Converting file number to OS filehandle
	hComm=get_osfhandle(fd);
	if (hComm==INVALID_HANDLE_VALUE){
		fprintf(stderr,"get_osfhandle error.\n");
		return -1;
	}
	//Setting up timer
	// get ticks per second
	QueryPerformanceFrequency(&frequency);
	// start timer
	QueryPerformanceCounter(&t1);
	do{
		if (ClearCommError(hComm, &comm_errors, &comm_stat)){
			if (comm_stat.cbInQue)
				return 1;
		}
		else
			return -1;
		//Check timer
		QueryPerformanceCounter(&t2);
		//compute time in usec and check timeout
    		if ( (t2.QuadPart - t1.QuadPart) * 1000000 / frequency.QuadPart > us )
			return 0;	
	}while (1);
#endif
}

int tty_poll(int dev)
{
	int bytes;
	ioctl(dev, FIONREAD, &bytes);
#ifdef  HAVE_DEBUG
	printf("ioctl(dev_sm, FIONREAD)=%i\n", bytes);
#endif
	return bytes;
}

int rtty_bps_to_const(int bps)
{
	static int bitrates[]={B50,50,B75,75,B110,110,B134,134,B150,150,B200,200,
		B300,300,B600,600,B1200,1200,B1800,1800,B2400,2400,B4800,4800,
		B9600,9600,B19200,19200,B38400,38400,B57600,57600,B115200,115200,
		B230400,230400,B460800,460800,B500000,500000,B576000,576000,
		B921600,921600,B1000000,1000000,B1152000,1152000,B1500000,1500000,
		B2000000,2000000};
	int i,bpsconst;
	bpsconst=0;
	for(i=0;i<sizeof(bitrates);i+=2)
		if(bitrates[i+1]==bps) {
		bpsconst=bitrates[i];
		break;
	}
	return bpsconst;
}

int rtty_set(int fd, int bpsconst)
{
	struct termios opts;
	tcgetattr(fd,&opts);
	cfmakeraw(&opts);
	opts.c_cflag = CS8|CREAD|CLOCAL;
	cfsetispeed(&opts,bpsconst);
	cfsetospeed(&opts,bpsconst);
	tcsetattr(fd, TCSADRAIN, &opts);
	//tcgetattr(fd,&opts);
	//printf("spd=%i\n",cfgetospeed(&opts));
	return 0;
}

typedef struct {
	int type;
	int len;
	int offset;
	int chksum;
	int valid;
	uint8_t data[256];
}ihex_line;

char *ihex_read (char *fname, int *len) {
	struct stat st;
	int r;
	int l;
	int fd;
	char *bfr;
	r = stat(fname, &st);
	if(r) return 0;
	l = st.st_size;
	bfr = (char*)malloc(l + 1);
	fd = open(fname, O_RDONLY);
	read(fd, bfr, l);
	bfr[l] = 0;
	if(len) *len = l;
	return bfr;
}

int ihex_get_line (char *bfr, int len, int *start, int *end) {
	int i, r, s, e;
	r = 0;
	s = 0;
	e = 0;
	for(i = *start; i < len; i++) {
		if(r == 0 && bfr[i] == ':') {
			r = 1;
			s = i;
		}
		if(r != 0 && bfr[i] < ' ') {
			e = i;
			break;
		}
	}
	//for(i = s; i < e; i++) write(0, &bfr[i], 1); printf("--\n");
	if(r != 0) {
		*start = s;
		*end = e;
		return e - s;
	} else {
		return 0;
	}
}

int ihex_parse_line (ihex_line *line, char *bfr, int len) {
	int i, c, offl, offh;
	//for(i = 0; i < len; i++) write(0, &bfr[i], 1); printf("--\n");
	line->len = HEX(&bfr[1]);
	offh = HEX(&bfr[3]);
	offl = HEX(&bfr[5]);
	line->offset = (offh << 8) + offl;
	line->type = HEX(&bfr[7]);
	line->chksum = HEX(&bfr[line->len * 2 + 9]);
	c = line->len + offh + offl + line->type;
	for(i = 0; i < line->len; i++) {
		line->data[i] = HEX(&bfr[i*2 + 9]);
		c += line->data[i];
	}
	//printf("cs=%02X/%02X\n", c & 0xFF, line->chksum);
	line->valid = ((c + line->chksum) & 0xFF) == 0 ? 1 : 0;
	return 0;
}

int ihex_parse (char *fname, uint8_t **data, int *length, unsigned *base) {
	uint32_t base_addr;
	uint32_t offset;
	uint32_t off_min;
	uint32_t off_max;
	uint32_t addr_max;
	int r, f;
	int len;
	int start, end;
	char *bfr;
	ihex_line line;
	bfr = ihex_read (fname, &len);
	if(!bfr) return -1;
	start = 0;
	end = 0;
	f = 0;
	off_max = 0;
	off_min = ~0;
	addr_max = 0;
	do {
		r = ihex_get_line (bfr, len, &start, &end);
		ihex_parse_line (&line, &bfr[start], end - start);
		//printf("type=%i, offset=%04X, len=%i, valid=%i\n", line.type, line.offset, line.len, line.valid);
		if(line.type == 4) {
			base_addr = (line.data[0] << 24) + (line.data[1] << 16);
			f ++;
		}
		if(line.type == 0) {
			if(off_min > line.offset ) off_min = line.offset;
			if(off_max < line.offset ) {
				off_max = line.offset;
				addr_max = line.offset + line.len;
			}
		}
		start = end;
	} while (r);
	if(f > 1) {
		printf("too many sections\n");
		return -1;
	}
	*length = addr_max - off_min;
	*base = base_addr + off_min;
	printf("load %i bytes @ %08X\n", *length, *base);
	*data = (uint8_t*)malloc(*length);
	start = 0;
	end = 0;
	do {
		r = ihex_get_line (bfr, len, &start, &end);
		ihex_parse_line (&line, &bfr[start], end - start);
		if(line.type == 0) {
			memcpy(*data + line.offset - off_min, line.data, line.len);
		}
		start = end;
	} while (r);
	return 0;
}

int cmdarg(int argc,char ** argv,char *s)
{
	int i;
	for(i=0;i<argc;i++) {
		if(!strcmp(argv[i],s))return i;
	}
	return -1;
}

void print_hex (uint8_t *data, int addr_base, int len) {
	int i, j;
	for(i = addr_base&~0x0F; i < (addr_base + len); i += 16) {
		printf("%04X\t", i);
		for(j = 0; j < 16; j++) {
			if( ((i + j) >= addr_base) && ((i + j) < (addr_base + len)) )printf("%02X ", *(data++));
			else printf("   ");
		}
		printf("\n");
	}
}

int uart_read(int dev, char *bfr, int len, int timeout) {
	int r;
	while(len) {
		r = tty_select(dev, timeout);
		if(r == 0) break;
		r = read(dev, bfr, len);
		len -= r;
		bfr += r;
	}
	//printf("len=%i\n", len);
	return len;
}

int mdr_uart_sync(int dev) {
	char s[8];
	int i, r;
	tcflush(dev, TCIOFLUSH);
	for(i = 5000; i > 0; i--) {
		write(dev, "\0", 1);
		r = tty_select(dev, 1000);
		//printf("sync poll = %i\n", r);
		if(r>0) break;
	}
	if(i == 0) return -1;
	r = read(dev, s, 1);
	//printf("[]%i\n", s[0]);
	if(s[0] != '\015') return -1;
	r = tty_select(dev, 100000);
	if(r < 0) return -1;
	r = read(dev, s, 1);
	//printf("[]%i\n", s[0]);
	if(s[0] != '\012') return -1;
	r = tty_select(dev, 100000);
	if(r < 0) return -1;
	r = read(dev, s, 1);
	//printf("[]%i\n", s[0]);
	if(s[0] != '>') return -1;
	printf("sync OK\n");
	return 1;
}

int mdr_uart_baud(int dev, int br) {
	char s[8];
	int p;
	int i, r;
	write(dev, "B", 1);
	s[0] = br & 0xFF;
	s[1] = (br >> 8) & 0xFF;
	s[2] = (br >> 16) & 0xFF;
	s[3] = (br >> 24) & 0xFF;
	write(dev, &s, 4);
	usleep(100000);
	rtty_set(dev, rtty_bps_to_const(br));
	r = tty_select(dev, 1000000);
	//printf("r=%i\n", r);
	s[0] = 0;
	r = read(dev, s, 1);
	//printf("[]%i\n", s[0]);
	if(r < 0) return -1;
	if(s[0] != 'B') return -1;
	printf("set baud rate OK\n");
	return 1;
}

int mdr_uart_load(int dev, uint8_t *data, int len, int base_addr) {
	char s[8];
	int p;
	int i, r;
	write(dev, "L", 1);
	s[0] = base_addr & 0xFF;
	s[1] = (base_addr >> 8) & 0xFF;
	s[2] = (base_addr >> 16) & 0xFF;
	s[3] = (base_addr >> 24) & 0xFF;
	write(dev, &s, 4);
	s[0] = len & 0xFF;
	s[1] = (len >> 8) & 0xFF;
	s[2] = (len >> 16) & 0xFF;
	s[3] = (len >> 24) & 0xFF;
	write(dev, &s, 4);
	r = tty_select(dev, 1000000);
	//printf("r=%i\n", r);
	s[0] = 0;
	r = read(dev, s, 1);
	//printf("[]%i\n", s[0]);
	if(r < 0) return -1;
	if(s[0] != 'L') return -1;
	for(i = 0; i < len; i++) {
		write(dev, &data[i], 1);
	}
	r = tty_select(dev, 1000000);
	//printf("r=%i\n", r);
	s[0] = 0;
	r = read(dev, s, 1);
	//printf("[]%i\n", s[0]);
	if(r < 0) return -1;
	if(s[0] != 'K') return -1;
	return len;
}

int mdr_uart_run(int dev, int addr) {
	char s[8];
	int p;
	int i, r;
	write(dev, "R", 1);
	s[0] = addr & 0xFF;
	s[1] = (addr >> 8) & 0xFF;
	s[2] = (addr >> 16) & 0xFF;
	s[3] = (addr >> 24) & 0xFF;
	write(dev, &s, 4);
	r = tty_select(dev, 1000000);
	//printf("r=%i\n", r);
	s[0] = 0;
	r = read(dev, s, 1);
	//printf("[]%i\n", s[0]);
	if(r < 0) return -1;
	if(s[0] != 'R') return -1;
	return 1;
}

int mdr_uart(int dev, int baud, uint8_t *data, int len, int base) {
	int r;
	int base_addr;
	base_addr = base;
	mdr_uart_sync(dev);
	r = mdr_uart_baud(dev, baud);
	if(r == -1) {
		r = mdr_uart_baud(dev, baud);
		if(r == -1) {
			printf("failed to set baud rate\n");
			return -1;
		}
	}
	printf("upload loader to MCU RAM\n");
	while(len > 0) {
		r = mdr_uart_load(dev, data, len > 128 ? 128 : len, base_addr);
		if(r == -1) {
			printf("failed load\n");
			return -1;
		}
		printf("load @ %08X, %i bytes\n", base_addr, r);
		base_addr += r;
		data += r;
		len -= r;
	}
	printf("run UART loader\n");
	r = mdr_uart_run(dev, base);
	if(r == -1) {
		printf("failed to run program\n");
		return -1;
	}
	return 1;
}

int mdr_boot_erase(int dev) {
	char s[8];
	int p;
	int i, r;
	uint32_t x1, x2;
	write(dev, "E", 1);
	r = tty_select(dev, 1000000);
	//printf("r=%i\n", r);
	s[0] = 0;
	r = read(dev, s, 1);
	//printf("[]%i\n", s[0]);
	if(r < 0) return -1;
	if(s[0] != 'E') return -1;
	r = uart_read(dev, s, 8, 100000);
	if(r != 0) {
		printf("fail\n");
		return -1;
	}
	x1 = s[0] | (s[1] << 8) | (s[2] << 16) | (s[3] << 24);
	x2 = s[4] | (s[5] << 8) | (s[6] << 16) | (s[7] << 24);
	//printf("x1=%08X, x2=%08X\n", x1, x2);
	return 1;
}

int mdr_boot_addr(int dev, int addr) {
	char s[8];
	int p;
	int i, r;
	int cs;
	uint32_t x1, x2;
	write(dev, "A", 1);

	s[0] = addr & 0xFF;
	s[1] = (addr >> 8) & 0xFF;
	s[2] = (addr >> 16) & 0xFF;
	s[3] = (addr >> 24) & 0xFF;
	write(dev, &s, 4);
	cs = s[0] + s[1] + s[2] + s[3];
	r = tty_select(dev, 1000000);
	//printf("r=%i\n", r);
	r = uart_read(dev, s, 1, 100000);
	if(r != 0) {
		printf("timeout error\n");
		return -1;
	}
	if((cs & 0xFF) != (((int)s[0]) & 0xFF)) {
		printf("checksum error\n");
		return -1;
	}
	return 1;
}

int mdr_boot_run(int dev, int addr) {
	char s[8];
	int p;
	int i, r;
	int cs, cs_r;
	uint32_t x1, x2;
	write(dev, "R", 1);
	s[0] = addr & 0xFF;
	s[1] = (addr >> 8) & 0xFF;
	s[2] = (addr >> 16) & 0xFF;
	s[3] = (addr >> 24) & 0xFF;
	write(dev, &s, 4);
	r = tty_select(dev, 1000000);
	r = uart_read(dev, s, 1, 100000);
	if(r != 0) {
		printf("timeout error\n");
		return -1;
	}
	if(s[0] != 'R') {
		printf("fail\n");
		return -1;
	}
	return 1;
}

int mdr_boot_load(int dev, uint8_t *bfr) {
	char s[8];
	int p;
	int i, r;
	int cs;
	cs = 0;
	write(dev, "P", 1);
	for(i = 0; i < 256; i++) {
		write(dev, bfr, 1);
		cs += *bfr;
		bfr ++;
	}
	r = tty_select(dev, 1000000);
	//printf("r=%i\n", r);
	r = uart_read(dev, s, 1, 100000);
	if(r != 0) {
		printf("timeout error\n");
		return -1;
	}
	if((cs & 0xFF) != (((int)s[0]) & 0xFF)) {
		printf("checksum error\n");
		return -1;
	}
	return 1;
}

int mdr_boot(int dev, uint8_t *data, int len, int base_addr, uint8_t run_fw) {
	int r;
	printf("erase eeprom\n");
	r = mdr_boot_erase(dev);
	if(r < 0) return -1;
	printf("seting firmware base address\n");
	r = mdr_boot_addr(dev, base_addr);
	if(r < 0) return -1;
	printf("sending firmware to UART loader\n");
	len = (len + 0xFF) & ~0xFF;
	while(len) {
		printf("load @ %08X, %i bytes\n", base_addr, 256);
		r = mdr_boot_load(dev, data);
		if(r < 0) return -1;
		base_addr += 256;
		data += 256;
		len -= 256;
	}
	if(run_fw){
		printf("running firmware\n");
		r = mdr_boot_run(dev, base_addr);
		if(r < 0) return -1;
	}
	return 1;
}

void print_help(void){
	printf("Bootstrap loader for MDR32F9Q2I, К1986ВЕ92QI, 1986ВЕ9x microcontrollers.\n"
		"It uses factory boot loader burned in MCU boot sector to load next stage\n"
		"loader 1986_BOOT_UART.hex into RAM, which, in turn, can load custom\n "
		"firmware, burn it into EEPROM and run.\n"
		"\n"
		"Usage:\n"
		"-s 115200\n"
		"    set upload baud rate\n"
		"-d /dev/ttyUSB0\n"
		"    set serial device\n"
		"-b 1986_BOOT_UART.hex\n"
		"    set stage 2 loader hex image\n"
		"-f firmware_filename\n"
		"    set user custom firmware hex image\n"
		"-r\n"
		"    run firmware after burning\n"
		"All parameters are mandatory, except '-r'\n"
		"\n"
		"Example:\n"
		"./mdrbootstrap -s 115200 -d /dev/ttyUSB0 -b 1986_BOOT_UART.hex -f 1986BE9x_Demo.hex\n");

}

int main(int argc, char **argv)
{
	int bps = 9600, bpsconst;
	char *dev_fn;
	char *fw_fn;
	char *boot_fn;
	int dev;
	int r;
	struct timespec delay;
	struct timeval timev,timev0;
	uint8_t run_fw=0;

	r = cmdarg(argc,argv,"-r");
	if(r>0) {
		run_fw=1;
	}

	r = cmdarg(argc,argv,"-b");
	if(r>0) {
		boot_fn = argv[r+1];
	} else {
		print_help();
		printf("Parameter not specified: -b \n");
		exit(-1);
	}
	r = cmdarg(argc,argv,"-f");
	if(r>0) {
		fw_fn = argv[r+1];
	} else {
		print_help();
		printf("Parameter not specified: -f \n");
		exit(-1);
	}
	r = cmdarg(argc,argv,"-d");
	if(r>0) {
		dev_fn = argv[r+1];
	} else {
		print_help();
		printf("Parameter not specified: -d \n");
		exit(-1);
	}
	dev = open(dev_fn, O_RDWR | O_NONBLOCK);
	if(dev==-1) {
		printf("failed to open %s\n",argv[r+1]);
		return(-1);
	}
	r=cmdarg(argc,argv,"-s");
	if(r>0) {
		bps=atoi(argv[r+1]);
	}
	printf("bps=%i\n", bps);
	bpsconst=rtty_bps_to_const(bps);
	if(!bpsconst) {
		printf("no such a bitrate %i\n",bps);
		return(-1);
	}
	rtty_set(dev,rtty_bps_to_const(9600));
	ihex_parse (boot_fn, &boot_data, &boot_len, &boot_base);
	//print_hex(boot_data, boot_base, boot_len);
	printf("read .hex firmware\nbase=%08X, len=%i\n", boot_base, boot_len);
	ihex_parse (fw_fn, &fw_data, &fw_len, &fw_base);
	//print_hex(fw_data, fw_base, fw_len);
	printf("read .hex firmware\nbase=%08X, len=%i\n", fw_base, fw_len);
	r = mdr_uart(dev, bps, boot_data, boot_len, boot_base);
	if(r > 0) 
		r=mdr_boot(dev, fw_data, fw_len, fw_base, run_fw);
	close(dev);
	if (r>0)
		return 0;
	else
		return -1;
}

/*
./mdrbootstrap -s 115200 -d /dev/ttyUSB0 -b 1986_BOOT_UART.hex -f 1986BE9x_Demo.hex
*/
