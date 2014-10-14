#define _GNU_SOURCE
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stropts.h>
#include <stdint.h>
#include <stdlib.h>
#include <signal.h>
#include <envytools/nva.h>


/* compilation: gcc -o ppwr_counters_fake ppwr_counters_fake.c -lnva -lpciaccess */

int cnum = 0, idx;

void sig_handler(int s)
{
	printf("Exiting... (sig %i)\n", s);
	nva_wr32(cnum, 0x10a50c + idx * 0x10, 2);
	exit(0);
}

int main(int argc, char **argv)
{
	int freq = 1000, period, t_on, t_off, i;
	unsigned int perc;

	if (nva_init()) {
		fprintf (stderr, "PCI init failure!\n");
		return 1;
	}
	int c;
	while ((c = getopt (argc, argv, "c:f:")) != -1)
		switch (c) {
			case 'c':
				sscanf(optarg, "%d", &cnum);
				break;
			case 'f':
				sscanf(optarg, "%d", &freq);
				break;
		}
	if (cnum >= nva_cardsnum) {
		if (nva_cardsnum)
			fprintf (stderr, "No such card.\n");
		else
			fprintf (stderr, "No cards found.\n");
		return 1;
	}

	if (optind + 1 >= argc) {
		fprintf (stderr, "Usage: %s [-c cnum][-f pwm_freq] counter_index percent\n", argv[0]);
		return 1;
	}

	sscanf (argv[optind], "%d", &idx);
	sscanf (argv[optind + 1], "%d", &perc);

	if (idx > 6) {
		fprintf (stderr, "Counter_index is too big. Valid values are < 7\n");
		return 1;
	}
	if (perc > 100)
		perc = 100;

	/* calculate the timings */
	period = 1000000 / freq;
	t_on = perc * period / 100;
	t_off = period - t_on;

	printf("period = %u, t_on = %u, t_off = %u\n", period, t_on, t_off);

	/* catch all signals */
	for (i = 0; i < 32; i++)
		if (i != 28)
			signal(i, sig_handler);

	/* PWM! */
	while (1) {
		/* make the counter count every tick */
		nva_wr32(cnum, 0x10a50c + idx * 0x10, 3);

		usleep(t_on);

		/* stop counting */
		nva_wr32(cnum, 0x10a50c + idx * 0x10, 0);

		usleep(t_off);
	}

	return 0;
}
