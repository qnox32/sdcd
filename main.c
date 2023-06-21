#include <kernel.h>
#include <loadfile.h>
#include <stdio.h>
#include <malloc.h>
#include <debug.h>
#include <stdint.h>
#include <sbv_patches.h>
#include <fcntl.h>
#include <dirent.h>
#include <sifrpc.h>
#include <libpad.h>
#include <errno.h>

#define RPC_ID 0x1a1a1a1b
#define MSG_BUFFER_SIZE 0x80000  /* 512KB */

#define WRITE_SIZE 10240 /* 10KB - 20 sectors */

/* printf to screen and over the network */
#define D_PRINTF(format, args...) do {  scr_printf(format, ##args); \
                                        printf(format, ##args); } while(0)

/* USB log file descriptor */
static int usb_log_fd = -1;
static char usb_log_name[256];

/* pad globals */
static char pad_buf[256] __attribute__((aligned(64)));
static struct padButtonStatus pad_btn_status;
static int pad_new_state = 0;
static int pad_old_state = 0;

/* rpc globals */
static struct t_SifRpcServerData cb_srv;
static struct t_SifRpcDataQueue cb_queue;
static unsigned char rpc_server_stack[0x1800] __attribute__((aligned (16)));
static unsigned char cb_rpc_buffer[128] __attribute__((__aligned__(64)));
static int rpc_server_thread_id;
ee_thread_t rpc_thread;

/* rpc msg buffer */
static char *msg_buffer;
static uint32_t msg_buffer_len = 0;

/* module defs */
extern unsigned char iomanX_irx[];
extern unsigned int size_iomanX_irx;

extern unsigned char fileXio_irx[];
extern unsigned int size_fileXio_irx;

extern unsigned char usbd_irx[];
extern unsigned int size_usbd_irx;

extern unsigned char bdm_irx[];
extern unsigned int size_bdm_irx;

extern unsigned char bdmfs_fatfs_irx[];
extern unsigned int size_bdmfs_fatfs_irx;

extern unsigned char usbmass_bd_irx[];
extern unsigned int size_usbmass_bd_irx;

extern unsigned char padman_irx[];
extern unsigned int size_padman_irx;

extern unsigned char sio2man_irx[];
extern unsigned int size_sio2man_irx;

extern unsigned char mx4sio_bd_rpc_irx[];
extern unsigned int size_mx4sio_bd_rpc_irx;

extern unsigned char mx4sio_bd_rpc_v_irx[];
extern unsigned int size_mx4sio_bd_rpc_v_irx;

static uint32_t mx4sio_mod_id;

/* modules */
struct iop_mod_t{
    char *name;
    uint8_t *irx;
    uint32_t *size;
};

struct iop_mod_t iop_mods[8] = {
    {"iomanX",          iomanX_irx,     &size_iomanX_irx},
    {"fileXio",         fileXio_irx,    &size_fileXio_irx},
    {"usbd",            usbd_irx,       &size_usbd_irx},
    {"bdm",             bdm_irx,        &size_bdm_irx},
    {"bdmfs_fatfs",     bdmfs_fatfs_irx,&size_bdmfs_fatfs_irx},
    {"usbmass_bd_irx",  usbmass_bd_irx, &size_usbmass_bd_irx},
    {"sio2man_irx",     sio2man_irx,    &size_sio2man_irx},
    {"padman_irx",      padman_irx,     &size_padman_irx}
};

static int modules_init()
{
    int rv;

    for (int i = 0; i < 8; i++) {
        rv = SifExecModuleBuffer((void*)iop_mods[i].irx, *iop_mods[i].size, 0, NULL, NULL);
        if (rv < 0) {
            printf("failed to load %s, %i\n", iop_mods[i].name, rv);
            return -1;
        }
    }
    
    return 0;
}


/* rpc */
static void *rpc_receiver(int func, char *data, int size)
{
    int rv;

    /* NOTE: BDMFS_FATFS uses a global lock across devices. To avoid deadlocking 
     * RPC messages are buffered until all read/write tests have completed */
    rv = snprintf((msg_buffer + msg_buffer_len), (MSG_BUFFER_SIZE - msg_buffer_len), data);
    
    if (rv > 0) {
        msg_buffer_len += rv;
    }
    
    return;
}

static void rpc_server_thread(void *arg)
{
	(void)arg;

    SifInitRpc(0);

	SifSetRpcQueue(&cb_queue, GetThreadId());
	SifRegisterRpc(&cb_srv, RPC_ID, &rpc_receiver, cb_rpc_buffer, NULL, NULL, &cb_queue);
    SifRpcLoop(&cb_queue);
}

static int rpc_init()
{
    int rv;
	
    msg_buffer = malloc(sizeof(char) * MSG_BUFFER_SIZE);
    if (msg_buffer == NULL) {
        printf("failed to alloc msg_buffer\n");
        return -1;
    }

    rpc_thread.attr = 0;
	rpc_thread.option = 0;
	rpc_thread.func = &rpc_server_thread;
	rpc_thread.stack = rpc_server_stack;
	rpc_thread.stack_size = sizeof(rpc_server_stack);
	rpc_thread.gp_reg = &_gp;
	rpc_thread.initial_priority = 0x60;
	
    rpc_server_thread_id = CreateThread(&rpc_thread);
    if (rpc_server_thread_id < 0) {
        printf("failed to create RPC thread, %i\n", rpc_server_thread_id);
        return -1;
    }

    rv = StartThread(rpc_server_thread_id, NULL);
    if (rv < 0) {
        printf("failed to start RPC thread, %i\n", rv);
        return -1;
    }

    return 0;
}


/* pads */
static int pad_init()
{
    int rv;

    padInit(0);

    rv = padPortOpen(0, 0, pad_buf);
    if (rv == 0) {
        printf("failed to open port\n");
        return -1;
    }

    return 0;
}

static void pad_update()
{
    int state = padGetState(0, 0);

    while (state != PAD_STATE_STABLE) {
        state = padGetState(0, 0);
    }

    state = padRead(0, 0, &pad_btn_status);

    if (state != 0) {
        pad_new_state = (0xFFFF ^ pad_btn_status.btns) & ~pad_old_state;
        pad_old_state = pad_btn_status.btns;
    }
}


/* tests */
static int test_proc_read()
{
    int rv;
    int fd;
    char *buffer = NULL;

    D_PRINTF("Starting read test\n");

    /* USB loaded first, MX4SIO *should* be mass1 */
    fd = open("mass1:zero.bin", O_RDONLY);
    if (fd <= 0) {
        D_PRINTF("Could not find zero.bin\n");
        return -1;
    }
    
    /* malloc 1MB */
    buffer = malloc(1048576);
    if (buffer == NULL) {
        D_PRINTF("failed to malloc test file buffer\n");
        return -1;
    }

    /* try to read 1MB from zeros.bin */
    rv = read(fd, buffer, 1048576);
    if (rv < 0) {
        D_PRINTF("failed to read zeros.bin\n");
    } else {
        D_PRINTF("Read OK\n");
    }
    
    close(fd);
    free(buffer);

    return 0;
}

static int test_proc_write()
{
    int rv;
    int fd;
    uint8_t *buffer = NULL;

    D_PRINTF("Starting write test\n");

    /* malloc 10KB */
    buffer = malloc(WRITE_SIZE);
    if (buffer == NULL) {
        D_PRINTF("failed to malloc test file buffer\n");
        return -1;
    }

    /* populate buffer with pattern */
    for (int i = 0; i < WRITE_SIZE; i++) {
        if (i % 2 == 0) {
            buffer[i] = 0xAA;
        } else {
            buffer[i] = 0xBB;
        }
    }

    /* USB loaded first, MX4SIO *should* be mass1 */
    fd = open("mass1:sdcard_write_test.bin", O_WRONLY | O_CREAT | O_TRUNC);
    if (fd <= 0) {
        D_PRINTF("Could not create test file on sdcard\n");
        free(buffer);
        return -1;
    }

    /* begin write */
    rv = write(fd, buffer, WRITE_SIZE);
    if (rv <= 0) {
        D_PRINTF("Write failed, %i\n", rv);
        free(buffer);
        close(fd);
        return -1;
    }

    /* close file to ensure contents are written */
    close(fd);
    fd = -1;

    /* clear buffer */
    memset(buffer, 0, WRITE_SIZE);

    /* try reopening file */
    fd = open("mass1:sdcard_write_test.bin", O_RDONLY);
    if (fd <= 0) {
        D_PRINTF("Could not reopen test file on sdcard, %i\n", errno);
        free(buffer);
        return -1;
    }

    /* read data back */
    rv = read(fd, buffer, WRITE_SIZE);
    if (rv <= 0) {
        D_PRINTF("Could read test file on sdcard, %i\n", rv);
        free(buffer);
        close(fd);
        return -1;
    }

    /* verify pattern */
    for (int i = 0; i < WRITE_SIZE; i++) {
        if (i % 2 == 0) {
            if (buffer[i] != 0xAA) {
                D_PRINTF("Pattern mismatch @%i - exp: 0xAA, got: %i\n", i, buffer[i]);
                rv = -1;
            }
        } else {
            if (buffer[i] != 0xBB) {
                D_PRINTF("Pattern mismatch @%i - exp: 0xBB, got: %i\n", i, buffer[i]);
                rv = -1;
            }
        }      
    }

    if (rv == -1) {
        D_PRINTF("Write FAILED\n");
    } else {
        D_PRINTF("Write OK\n");
    }
    
    free(buffer);
    close(fd);

    return 0;
}

/* main test procedure */
static int test_start(int verbose, int type)
{
    int rv;

    scr_clear();

    D_PRINTF("Loading MX4SIO module\n");

    /* load appropriate mx4sio module */
    if (verbose == 0) {
        mx4sio_mod_id = SifExecModuleBuffer(mx4sio_bd_rpc_irx, size_mx4sio_bd_rpc_irx, 0, NULL, NULL);
        if (mx4sio_mod_id < 0) {
            D_PRINTF("failed to load mx4sio module, abort.\n");
            return -1;
        }
    } else {
        rv = SifExecModuleBuffer(mx4sio_bd_rpc_v_irx, size_mx4sio_bd_rpc_v_irx, 0, NULL, NULL);
        if (rv < 0) {
            D_PRINTF("failed to load mx4sio module\n");
            SleepThread();
        }
    }
    
    /* give mx4sio time to init */
    nanosleep((const struct timespec[]){{2, 0}}, NULL);

    switch (type)
    {
    case 0:
        test_proc_read();
        break;
    case 1:
        test_proc_write();
        break;
    case 3:
        test_proc_read();
        test_proc_write(); 
        break;
    default:
        break;
    }

    /* create log file on USB */
    for (int i = 0; i < 999; i++) {
        if (verbose) { 
            rv = snprintf(usb_log_name, 256, "mass0:sdcard_log_verbose_%i_%i.txt", type, i);
        } else {
            rv = snprintf(usb_log_name, 256, "mass0:sdcard_log_%i_%i.txt", type, i);
        }

        if (rv < 0)
            break;

        rv = access(usb_log_name, 0);
        if (rv != 0) {
            usb_log_fd = open(usb_log_name, O_RDWR | O_CREAT);
            if (usb_log_fd > 0) {
                break;
            }
        }
    }

    /* write log file */
    if (usb_log_fd > 0) {
        D_PRINTF("\nWriting log to %s\n", usb_log_name);
        write(usb_log_fd, msg_buffer, msg_buffer_len);
        close(usb_log_fd);
        usb_log_fd = -1;
    }

    /* reset msg buffer */
    memset(msg_buffer, '\0', MSG_BUFFER_SIZE);
    msg_buffer_len = 0;

    return 0;
}

static void iop_reset()
{
    int rv;
    
    fileXioExit();
    SifExitIopHeap();
    SifLoadFileExit();
    SifExitRpc();

    while (!SifIopReset("", 0));

    while (!SifIopSync());

    SifLoadFileInit();
    SifInitIopHeap();

    sbv_patch_enable_lmb();
    sbv_patch_disable_prefix_check();

    rv = modules_init();
    if (rv != 0) {
        printf("modules_init failed, abort.\n");
        SleepThread();
    }

    rv = pad_init();
    if (rv != 0) {
        printf("pad_init failed, abort.\n");
        SleepThread();
    }

    fileXioInit();
    init_scr();
}

/* menus */
static void menu_main_draw()
{
    scr_clear();
    scr_printf("SD Card Compatibility Debugger v0.4\n");

    scr_printf("(X) Begin Read Test\n");
    scr_printf("(O) Begin Read Test (Verbose)\n");
    scr_printf("\n");
    scr_printf("WARNING: Possible data corruption, only use if your card passes read tests\n");
    scr_printf("(SEL) + (X) Begin Write Test\n");
    scr_printf("(SEL) + (O) Begin Write Test (Verbose)\n");
    scr_printf("\n");
    scr_printf("(^) Exit");
}

static void menu_post_draw()
{
    scr_printf("Returning to menu....");
    nanosleep((const struct timespec[]){{5, 0}}, NULL);
    /* reset iop to unload MX4SIO driver */
    iop_reset();
}

int main()
{
    int rv;

    rv = rpc_init();
    if (rv != 0) {
        printf("rpc_init failed, abort.\n");
        SleepThread();
    }

    iop_reset();
    
    /* draw initial menu */
    menu_main_draw();

    while (1) {

        pad_update();

        if (!(pad_new_state & PAD_SELECT) && (pad_new_state & PAD_CROSS)) {
            test_start(0, 0);
            /* draw post test options */
            menu_post_draw();
            menu_main_draw();
        }

        if (!(pad_new_state & PAD_SELECT) && (pad_new_state & PAD_CIRCLE)) {
            test_start(1, 0);
            menu_post_draw();
            menu_main_draw();
        }

        if ((pad_new_state & PAD_SELECT) && (pad_new_state & PAD_CROSS)) {
            test_start(0, 1);
            menu_post_draw();
            menu_main_draw();
        }

        if ((pad_new_state & PAD_SELECT) && (pad_new_state & PAD_CIRCLE)) {
            test_start(1, 1);
            menu_post_draw();
            menu_main_draw();
        }

        if (pad_new_state & PAD_TRIANGLE) {
            scr_printf("\nExiting...\n\n");
            return 0;
        }
    }

    return 0;
}