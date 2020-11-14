#!/bin/sh

set -e

(cd thumbulator; sh compile)
(cd mecrisp-stellaris-source
	(cd palanqin; make)
	(cd palanqin-ra; make)
)
(cd palanqin; sh buildcore)
(cd palanqin-ra; sh buildcore)
