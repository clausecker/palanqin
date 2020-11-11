/* Copyright (c) 2020 Robert Clausecker <fuz@fuz.su> */
/* minimal fake libc for Palanqin images */

#include <stdio.h>
#include <stdarg.h>
#include <time.h>

/* dummy files */
FILE *stdin = NULL, *stdout = NULL;

/*
 * Read a character from the console, translating CR int LF.
 * An echo is produced, too.
 */
extern int
getchar(void)
{
	register int c asm("r0");

	asm volatile (".inst.n 0xb703" : "=r"(c));

	if (c == '\r')
		c = '\n';

	return (putchar(c));
}

/*
 * Write a character to the console.  Print a carriage return before
 * a newline character.
 */
extern int
putchar(int c)
{
	register int cc asm("r0");

	if (c == '\n') {
		cc = '\r';
		asm volatile (".inst.n 0xb702" :: "r"(cc));
	}

	cc = c;
	asm volatile (".inst.n 0xb702" :: "r"(cc));

	return (c);
}

/* dummy function */
extern int
fflush(FILE *f)
{
	(void)f;
	return (0);
}

extern char *
fgets(char *buf, int len, FILE *f)
{
	int i = 0;

	(void)f;
	len--;

	while (i < len) {
		buf[i] = getchar();

		if (buf[i++] == '\n')
			break;
	}

	buf[i] = '\0';
	return (buf);
}

extern int
puts(const char *s)
{
	size_t i;

	for (i = 0; s[i] != '\0'; i++)
		putchar(s[i]);

	putchar('\n');

	return (0);
}

/* a very simple printf implementation.  Just %s and %d */
extern int
printf(const char *fmt, ...)
{
	va_list ap;
	size_t i = 0, j, m;
	int d, n = 0;
	char *s, numbuf[10];

	va_start(ap, fmt);

	while (fmt[i] != '\0') {
		if (fmt[i] != '%') {
			putchar(fmt[i++]);
			n++;
			continue;
		}

		i++; /* advance past directive */
		switch (fmt[i++]) {
		case 's':
			s = va_arg(ap, char *);
			for (j = 0; s[j] != '\0'; j++)
				putchar(s[j]);
			n += (int)j;
			continue;

		case 'd':
			d = va_arg(ap, int);
			if (d < 0) {
				putchar('-');
				n++;
				d = -d;
			}
			m = 0;
			do {
				numbuf[m++] = d % 10 + '0';
				d /= 10;
			} while (d != 0);
			for (j = 0; j < m; j++) {
				putchar(numbuf[m-j-1]);
			}
			n += (int)m;
			continue;

		case 'c':
			putchar(va_arg(ap, int));
			n++;
			continue;

		default:
			putchar('%');
			putchar(fmt[i-1]);
			n += 2;
			continue;
		}
	}

	return (n);
}

extern time_t
time(time_t *tloc)
{
	return (*tloc = 1337);
}

extern char *
strcpy(char *dest, const char *src)
{
	size_t i = 0;

	do dest[i] = src[i];
	while (src[i++] != '\0');
	return (dest);
}

extern void *
memcpy(void *dest, const void *src, size_t n)
{
	size_t i;
	char *d = dest;
	const char *s = src;

	for (i = 0; i < n; i++)
		d[i] = s[i];

	return (dest);
}

extern int
strcmp(const char *a, const char *b)
{
	size_t i = 0;

	do {
		if (a[i] != b[i])
			return ((a[i] > b[i]) - (a[i] < b[i]));
	} while (a[i++] != '\0');

	return (0);
}

extern int
strncmp(const char *a, const char *b, size_t n)
{
	size_t i;

	for (i = 0; i < n; i++)
		if (a[i] == '\0' || a[i] != b[i])
			return ((a[i] > b[i]) - (a[i] < b[i]));

	return (0);
}

extern int
memcmp(const void *aa, const void *bb, size_t n)
{
	size_t i;
	const char *a = aa, *b = bb;

	for (i = 0; i < n; i++)
		if (a[i] != b[i])
			return ((a[i] > b[i]) - (a[i] < b[i]));

	return (0);
}

extern int
toupper(int c)
{
	if (c >= 'a' && c <= 'z')
		return (c - 0x20);
	else
		return (c);
}

extern int
tolower(int c)
{
	if (c >= 'A' && c <= 'Z')
		return (c + 0x20);
	else
		return (c);
}

extern int
isspace(int c)
{
	return (c == ' ' || c == '\f' || c == '\n' || c == '\r' || c == '\t' || c == '\v');
}

extern void _Noreturn
exit(int status)
{
	register int level asm("r0");

	for (;;) {
		level = status;
		asm volatile (".inst.n 0xb700" :: "r"(level));
	}
}

extern void _Noreturn
_start(void)
{
	char program[] = "palanqin";
	char *argv[] = { program, NULL };
	char *envp[] = { NULL };
	extern int main(int, char *[], char *[]);
	extern int edata, end;
	int *p = &edata;

	/* zero out bss */
	while (p < &end)
		*p++ = 0;

	exit(main(0, argv, envp));
}
