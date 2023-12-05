#******************************************************************************
#* @file    Makefile
#* @author  ARMCNC site:www.armcnc.net github:armcnc.github.io
#******************************************************************************


ifneq ($V,1)
Q ?= @
endif

DEBUG	= -O3
CC	?= gcc
INCLUDE	= -I/usr/local/include
CFLAGS	= $(DEBUG) -Wall $(INCLUDE) -Winline -pipe $(EXTRA_CFLAGS)

LDFLAGS	= -L/usr/local/lib
LDLIBS    = -lwiringPi -lwiringPiDev -lpthread -lm -lcrypt -lrt

SRC	=	armcnc_driver.c

OBJ	=	$(SRC:.c=.o)

BINS	=	$(SRC:.c=)

all:
	$(BINS)

armcnc_driver:	armcnc_driver.o
	$Q echo [link]
	$Q $(CC) -o $@ armcnc_driver.o $(LDFLAGS) $(LDLIBS)

.c.o:
	$Q echo [CC] $<
	$Q $(CC) -c $(CFLAGS) $< -o $@

clean:
	$Q echo "[Clean]"
	$Q rm -f $(OBJ) *~ core tags $(BINS)

tags:	$(SRC)
	$Q echo [ctags]
	$Q ctags $(SRC)

depend:
	makedepend -Y $(SRC)
