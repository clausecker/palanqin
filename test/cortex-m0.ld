ENTRY(_start);

SECTIONS {
	. = 0x0;

	.text : {
		LONG(0x1000); /* initial stack pointer */
		LONG(_start); /* entry point */
		*(.text.startup);
		*(.text);
	}

	etext = .;

	.data : {
		*(.data);
		*(.rodata);
		. = ALIGN(2);
	}

	edata = .;

	.bss : {
		*(.bss);
	}

	. = ALIGN(4);
	end = .;
}
