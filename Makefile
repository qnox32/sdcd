EE_BIN = sdcd.elf
EE_OBJS = main.o
EE_LIBS = -lpad -lfileXio -ldebug -lpatches

# Add embedded IRX files
EE_IRX_FILES=\
	iomanX.irx \
	fileXio.irx \
	usbd.irx \
	bdm.irx \
	bdmfs_fatfs.irx \
	usbmass_bd.irx \
	sio2man.irx \
	padman.irx \
	mx4sio_bd_rpc.irx \
	mx4sio_bd_rpc_v.irx \

EE_IRX_OBJS = $(addsuffix _irx.o, $(basename $(EE_IRX_FILES)))
EE_OBJS += $(EE_IRX_OBJS)

# Where to find the IRX files
vpath %.irx $(PS2SDK)/iop/irx/

# Rule to generate them
%_irx.o: %.irx
	@bin2c $< $*_irx.c $*_irx
	@mips64r5900el-ps2-elf-gcc -c $*_irx.c -o $*_irx.o

all: $(EE_BIN)

clean:
	rm -f -r $(EE_OBJS) $(EE_BIN) *_irx.c

include $(PS2SDK)/samples/Makefile.pref
include $(PS2SDK)/samples/Makefile.eeglobal
