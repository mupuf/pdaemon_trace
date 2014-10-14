#define _GNU_SOURCE
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stropts.h>
#include <stdint.h>
#include <stdlib.h>
#include <envytools/nva.h>
#include <sched.h>

/* compilation: gcc -o pwr_read pwr_read.c -lnva -lpciaccess -lpthread */
/* must be run alongside the nvidia's proprietary driver, as root */

/* WARNING: Only the ina3221 case has been tested, use nvbios to check which
 * power sensor you have (if you do have one!).
 */

/* config */
uint8_t resistor[3] = { 5, 5, 5 };
uint8_t ina3221_addr = 0x40;
uint8_t ina219_addresses[3] = { 0x48, 0x49, 0x4a };

struct power_lane {
	double V;
	double I;
	double W;
};

int i2c_rd16(int fd, uint8_t reg, uint16_t *val)
{
	uint8_t data[3];
	data[0] = reg;
	if (write(fd, data, 1) != 1)
		return errno;
	if (read(fd, data, 2) != 2)
		return errno;
	*val = data[0] << 8 | (data[1]);
	return 0;
}

int i2c_wr16(int fd, uint8_t reg, uint16_t val)
{
	uint8_t data[3];
	data[0] = reg;
	data[1] = (val & 0xff);
	data[2] = (val >> 8);
	if (write(fd, data, 3) != 3)
		return errno;
	return 0;
}

int ina219_poll_power_lane(int fd, uint8_t id, uint8_t resistor, struct power_lane *lane)
{
	uint16_t vbus, vshunt;

	if (id > 2)
		return -EINVAL;

	if (ioctl(fd, I2C_SLAVE, ina219_addresses[id]) < 0) {
		perror("Failed to acquire bus access and/or talk to slave");
		return -ENOENT;
	}

	if (i2c_rd16(fd, 1, &vshunt) || i2c_rd16(fd, 2, &vbus))
		return -EINVAL;

	lane->V = (vbus & 0xfff8) / 2000.0;
	lane->I = vshunt * 10 / (double)resistor / 1000.0;
	lane->W = lane->V * lane->I;

	return 0;
}

int find_ina219()
{
	struct power_lane lane;
	char filename[51];
	int fd, i;

	printf("Trying to find an INA219\n");
	for (i = 0; i < 20; i++) {
		snprintf(filename, sizeof(filename) - 1, "/dev/i2c-%i", i);
		filename[50] = '\0';

		printf("Trying device '%s'\n", filename);
		fd = open(filename, O_RDWR);
		if (fd < 0) {
			if (errno != ENOENT)
				perror("open failed");
			continue;
		}

		if (ina219_poll_power_lane(fd, 0, resistor[0], &lane)) {
			printf("Cannot read back from device 0 (0x%x)\n", ina219_addresses[0]);
			continue;
		}
		if (ina219_poll_power_lane(fd, 1, resistor[1], &lane)) {
			printf("Cannot read back from device 1 (0x%x)\n", ina219_addresses[1]);
			continue;
		}
		if (ina219_poll_power_lane(fd, 2, resistor[2], &lane)) {
			printf("Cannot read back from device 2 (0x%x)\n", ina219_addresses[2]);
			continue;
		}

		return fd;
	}

	printf("--> couldn't find an INA219\n\n");
	return -ENOENT;
}

int use_ina219()
{
	struct power_lane lane[3];
	int i;

	int fd = find_ina219();
	if (fd < 0)
		return -1;

	/* set the averaging factor to the max */
	//i2c_wr16(fd, 0x0, 0x7f27);

	while (1) {
		float W = 0;
		for (i = 0; i < 3; i++) {
			ina219_poll_power_lane(fd, i, resistor[i], &lane[i]);
			W += lane[i].W;
		}

		printf("P = %.03f W (", W);
		for (i = 0; i < 3; i++)
			printf("%i: %.02f W%s", i, lane[i].W, i==2?"":", ");
		printf(")\n");

		usleep(500000);
	}

	close(fd);
	return 0;
}

int find_ina3221()
{
	char filename[51];
	int fd, i = 0;
	uint16_t val;

	printf("Trying to find an INA3221\n");
	for (i = 0; i < 20; i++) {
		snprintf(filename, sizeof(filename) - 1, "/dev/i2c-%i", i);
		filename[50] = '\0';

		printf("Trying device '%s'\n", filename);
		fd = open(filename, O_RDWR);
		if (fd < 0) {
			if (errno != ENOENT)
				perror("open failed");
			continue;
		}

		if (ioctl(fd, I2C_SLAVE, ina3221_addr) < 0) {
			perror("Failed to acquire bus access and/or talk to slave");
			continue;
		}

		if (!i2c_rd16(fd, 0xff, &val) && val == 0x3220)
			return fd;

		printf("Invalid signature (0x%x)for the INA3221, trying next device.\n", val);
	}
	printf("--> couldn't find an INA3221\n\n");

	return -ENOENT;
}

int ina3221_poll_power_lane(int fd, uint8_t id, uint8_t resistor, struct power_lane *lane)
{
	uint16_t vbus, vshunt;

	if (id > 2)
		return -EINVAL;

	if (i2c_rd16(fd, 1 + (id * 2), &vshunt) || i2c_rd16(fd, 2 + (id * 2), &vbus))
		return -EINVAL;

	lane->V = vbus / 1000.0;
	lane->I = vshunt * 5.0 / (double)resistor / 1000.0;
	lane->W = lane->V * lane->I;

	return 0;
}

int use_ina3221()
{
	struct power_lane lane[3];
	int i;

	int fd = find_ina3221();
	if (fd < 0)
		return -1;

	/* set the averaging factor to the max */
	i2c_wr16(fd, 0x0, 0x7f27);

	while (1) {
		float W = 0;
		for (i = 0; i < 3; i++) {
			ina3221_poll_power_lane(fd, i, resistor[i], &lane[i]);
			W += lane[i].W;
		}

		printf("P = %.03f W (", W);
		for (i = 0; i < 3; i++)
			printf("%i: %.02f W%s", i, lane[i].W, i==2?"":", ");
		printf(")\n");

		usleep(500000);
	}

	close(fd);
	return 0;
}

typedef enum { false = 0, true = 1 } bool;
enum state {
	IDLE = 0,
	ADDR,
	REG,
	REG_STOP,
	RST_ADDR,
	DATA,
	ERROR
};

struct i2c_state {
	enum state st;

	/* last recorded state of sda and scl */
	uint8_t sda;
	uint8_t scl;

	/* current byte */
	uint8_t byte;
	uint8_t offset;

	/* addr, reg and data associated with the transaction */
	uint8_t addr;
	uint8_t reg;
	uint32_t data;
};

void i2c_init(struct i2c_state *i2c)
{
	i2c->st = IDLE;
	i2c->sda = 1;
	i2c->scl = 1;
	i2c->byte = 0;
	i2c->offset = 0;
	i2c->addr = 0;
	i2c->reg = 0;
	i2c->data = 0;
}

static char debugbuf[4096 * 100] = { 0 }; /* we can't use printf because it introduces jitter */
static uint32_t debugoff = 0;

bool i2c_add_data(struct i2c_state *i2c, uint8_t scl, uint8_t sda)
{
	bool ret = false;

	if (i2c->sda == sda && i2c->scl == scl)
		return false;

	//debugoff += snprintf(debugbuf + //debugoff, sizeof(debugbuf) - //debugoff, "%i: scl = %i sda = %i: ", i2c->st, scl, sda);

	if (i2c->scl && scl) {
		if (i2c->sda && !sda) {
			/* start */
			//debugoff += snprintf(debugbuf + //debugoff, sizeof(debugbuf) - //debugoff, "START: ");
			if (i2c->st == IDLE)
				i2c->st = ADDR;
			else if (i2c->st == REG_STOP)
				i2c->st = RST_ADDR;
			else {
				//debugoff += snprintf(debugbuf + //debugoff, sizeof(debugbuf) - //debugoff, "ERR");
				i2c->st = ERROR;
			}
			i2c->offset = 0;
			i2c->byte = 0;
			i2c->data = 0;
		} else if (!i2c->sda && sda) {
			/* stop */
			//debugoff += snprintf(debugbuf + //debugoff, sizeof(debugbuf) - //debugoff, "STOP: ");
			if (i2c->st == DATA && i2c->offset == 1) {
				i2c->st = IDLE;
				ret = true;
			} else {
				//debugoff += snprintf(debugbuf + //debugoff, sizeof(debugbuf) - //debugoff, "ERR, state == %i and offset = %i", i2c->st, i2c->offset);
				i2c->st = ERROR;
			}
		}
	} else if (!i2c->scl && scl) {
		if (i2c->offset == 8) {
			//debugoff += snprintf(debugbuf + //debugoff, sizeof(debugbuf) - //debugoff, "ACK (%i): ", sda);
			if (i2c->st == ADDR) {
				i2c->addr = i2c->byte;
				i2c->st = REG;
				//debugoff += snprintf(debugbuf + //debugoff, sizeof(debugbuf) - //debugoff, "Address = 0x%.02x ", i2c->addr);
			} else if (i2c->st == REG) {
				i2c->reg = i2c->byte;
				i2c->st = REG_STOP;
				//debugoff += snprintf(debugbuf + //debugoff, sizeof(debugbuf) - debugoff, "Reg = 0x%.02x ", i2c->reg);
			} else if (i2c->st == RST_ADDR) {
				if ((i2c->byte & 0xfe) != i2c->addr) {
					i2c->st = ERROR;
				}
				i2c->st = DATA;
			} else if (i2c->st == DATA) {
				i2c->data <<= 8;
				i2c->data |= i2c->byte;
			}

			i2c->offset = 0;
			i2c->byte = 0;
		} else {
			i2c->byte |= sda;
			//debugoff += snprintf(debugbuf + debugoff, sizeof(debugbuf) - debugoff, "bit %i = %i, byte = 0x%.02x", 7 - i2c->offset, sda, i2c->byte);
			i2c->offset++;
			if (i2c->offset < 8)
				i2c->byte <<= 1;

		}
	}
	//debugoff += snprintf(debugbuf + //debugoff, sizeof(debugbuf) - debugoff, "\n");

	i2c->sda = sda;
	i2c->scl = scl;

	return ret;
}

void print_debug()
{
	printf(debugbuf);
	debugoff = 0;
}

void process_set_affinity(int cpu)
{
	cpu_set_t  mask;
	CPU_ZERO(&mask);
	CPU_SET(cpu, &mask);
	if (!sched_setaffinity(0, sizeof(mask), &mask))
		perror("sched_setaffinity");
}

#include <sys/time.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>

volatile float power = 0;
volatile int cnum = 0;
volatile float pc_mem = 0, pc_core = 0, pc_vdec = 0, pc_unk = 0;

long long current_timestamp() {
	struct timeval te;
	gettimeofday(&te, NULL);
	unsigned long long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000;
	return milliseconds;
}

void * start_routine_perf(void *ptr)
{
	uint32_t pc_total, pc_total_prev = 0;
	float mem, p_mem, core, p_core, vdec, p_vdec, unk, p_unk;

	while (1) {
		/* usage */
		vdec = nva_rd32(cnum, 0x10a518) * 100.0;
		core = nva_rd32(cnum, 0x10a538) * 100.0;
		mem = nva_rd32(cnum, 0x10a548) * 100.0;
		unk = nva_rd32(cnum, 0x10a558)* 100.0;

		pc_total = nva_rd32(cnum, 0x10a578);
		vdec /= pc_total;
		core /= pc_total;
		mem /= pc_total;
		unk /= pc_total;

		if (pc_total < pc_total_prev) {
			pc_mem = p_mem;
			pc_core = p_core;
			pc_vdec = p_vdec;
			pc_unk = p_unk;
		}
		pc_total_prev = pc_total;
		p_mem = mem;
		p_core = core;
		p_vdec = vdec;
		p_unk = unk;

		usleep(10000);
	}
}

int nvd0_gpio_sense(int line)
{
	return !!(nva_rd32(cnum, 0x00d610 + (line * 4)) & 0x00004000);
}

static void force_temperature(uint8_t temp) {
	nva_wr32(cnum, 0x20008, 0x80008000 | ((temp & 0x3f) << 22));
}

static void unforce_temperature() {
	nva_wr32(cnum, 0x20008, 0x80000000);
}

void read_fsrm(uint8_t *div, uint8_t *fsrm)
{
	uint32_t reg = nva_rd32(cnum, 0x201e0);
	*div = reg & 0x1f;
	*fsrm = (reg & 0xff) >> 8;
}

uint64_t time_us() {
	struct timeval time;
	gettimeofday(&time, NULL);
	return time.tv_sec * 1e6 + time.tv_usec;
}

void * start_routine_freq(void *ptr)
{
	FILE *in;
	char *buffer = NULL;
	uint32_t core, hubk07, rop, mem, hubk06, hubk01, vdec;
	uint32_t pcie_conf, pcie_speed, pcie_lane;
	uint8_t vid, temp;
	uint64_t start = current_timestamp(), cur_time;
	uint8_t div, fsrm;
	int len;

	unforce_temperature();

	fprintf(stdout, "Time (ms), Power (W), Core (MHz), Hubk07 (Mhz), ROP (MHz), "
			"Memory (MHz), Hubk06 (Mhz), Hubk01 (Mhz), VDec (Mhz), "
			"Perf_VDec (%), Perf_Core (%), Perf_Memory (%), Perf_UNK (%), "
			"PCIE speed (MHz), PCIE lanes, VID, temperature (°C), "
			"FSRM div, FSRM ratio\n");
	do {
		if(!(in = popen("/root/src/nouveau/bin/nv_init -- -c \"config=NvBios=vbios.rom\" | grep \"\\-\\-\" | cut -d ' ' -f 10-", "r"))) // vbios.rom extracted with nvagetbios -s prom
			exit(1);

		/* pcie */
		pcie_conf = nva_rd32(cnum, 0x088088);
		pcie_speed = ((pcie_conf >> 16) & 3) * 2500;
		pcie_lane = (pcie_conf >> 20) & 0x1f;

		/* vid */
		vid = 0;
		vid |= nvd0_gpio_sense(3); vid <<= 1;
		vid |= nvd0_gpio_sense(2); vid <<= 1;
		vid |= nvd0_gpio_sense(1); vid <<= 1;
		vid |= nvd0_gpio_sense(0);

		/* change the temperature */
		cur_time = current_timestamp();
		/*temp = 70 + ((cur_time % 10000000) * 50 / 10e6);
		fprintf(stderr, "force temp to %u\n", temp);
		force_temperature(temp);*/

		read_fsrm(&div, &fsrm);

		fscanf(in,
		       "core %d MHz hubk07 %d MHz rop %d MHz memory %d MHz hubk06 %d MHz hubk01 %d MHz vdec %d MHz",
		       &core, &hubk07, &rop, &mem, &hubk06, &hubk01, &vdec);

		fprintf(stdout, "%6.llu, %6.2f, %4.u, %4.u, %4.u, %4.u, %4.u, %4.u, %4.u, "
				"%6.2f, %6.2f, %6.2f, %6.2f, %u, %3.u, %u, %u, %u, %u\n",
			cur_time - start,
			power, core, hubk07, rop, mem, hubk06, hubk01, vdec,
			pc_vdec, pc_core, pc_mem, pc_unk, pcie_speed, pcie_lane,
			vid, nva_rd32(cnum, 0x20400), div, fsrm
       		);
		fflush(stdout);

		pclose(in);

		usleep(10000);
	} while (1);
}

int main(int argc, char **argv)
{
	if (/* blob */ 1 ) {
		nva_init();
		uint8_t scl, sda;
		struct i2c_state i2c;
		struct power_lane lane[3];
		int polls = 0;
		int i;
		pthread_t thread_freq, thread_perf;

		signal(SIGINT, exit);

		i2c_init(&i2c);
		//process_set_affinity(2);

		/* get the frequencies */
		if (pthread_create(&thread_freq, NULL, start_routine_freq , NULL) < 0)
			perror("pthread_create");

		/* get the perf counters */
		if (pthread_create(&thread_perf, NULL, start_routine_perf , NULL) < 0)
			perror("pthread_create");

		do {
			uint32_t v = nva_rd32(cnum, 0xd054);
			if (v != 0x37)
				polls++;

			scl = !!(v & 0x10);
			sda = !!(v & 0x20);

			if (i2c_add_data(&i2c, scl, sda)) {
				/*debugoff += snprintf(debugbuf + debugoff, sizeof(debugbuf) - debugoff,
						    "%.02x:%.02x %s %.04x (status = %i, polls = %i)\n",
						     i2c.addr, i2c.reg,
						     (i2c.addr & 1)?"<==":"==>", i2c.data, i2c.st, polls);*/
				polls = 0;
				print_debug();

				if (i2c.reg == 0) {
					float W = 0;
					for (i = 0; i < 3; i++)
						W += lane[i].W;

					power = W;
					//printf("P = %.03f W (", W);
					/*for (i = 0; i < 3; i++)
						printf("%i: %.02f W%s", i, lane[i].W, i==2?"":", ");*/
					//printf(")\n");
				} else if (i2c.reg == 0x1 || i2c.reg == 0x3 || i2c.reg == 0x5) {
					int id = (i2c.reg - 1) / 2;
					lane[id].I = i2c.data * 5.0 / (double)resistor[id] / 1000.0;
				} else if (i2c.reg == 0x2 || i2c.reg == 0x4 || i2c.reg == 0x4) {
					int id = (i2c.reg - 1) / 2;
					lane[id].V = i2c.data / 1000.0;
					lane[id].W = lane[id].V * lane[id].I;
					//printf("lane[%i]: V = %.02f V, I = %.02f A, W = %.02f W\n", id, lane[id].V, lane[id].I, lane[id].W);
				}

			} else if (i2c.st == ERROR) {
				print_debug();
				//printf("Transaction error\n");
				i2c_init(&i2c);
			}
			sched_yield();
		} while (1);

		exit(1);
	} else {
		system("modprobe i2c-dev");
		system("modprobe nouveau");
	}

	use_ina219();
	use_ina3221();

	return 0;
}
