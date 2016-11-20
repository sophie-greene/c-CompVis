/*
#define	DEBUG
#define	TRACE
*/

#define	MAIN

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include	<gtk/gtk.h>
#include	<stdio.h>
#include	<stdlib.h>
#include	<string.h>
#include	<unistd.h>
#include	<errno.h>
#include	<signal.h>

#include	"interface.h"
#include	"support.h"
#include	"callbacks.h"
#include	"input-uvc.h"
#include	"device.h"
#include	"image.h"
#include	"configfile.h"
#include	"option.h"
#include	"debug.h"

#define	DEVICE	"/dev/video0"
#define	CONFIG	"uvccapture.conf"

#if	1
#define WIDTH	640
#define HEIGHT	480
#else
#define WIDTH	320
#define HEIGHT	240
#define WIDTH	2048
#define HEIGHT	2048
#define WIDTH	1024
#define HEIGHT	576
#endif

static	char	*Config;
static	char	*CamDevice;
static	int		FrameRate;
static	int		Width;
static	int		Height;

static	sigset_t SigMask;

#define FRAME_RATE			24


extern void
errno_exit(
	char	*s)
{
    fprintf (stderr, "%s error %d, %s\n",
	     s, errno, strerror (errno));
    exit (EXIT_FAILURE);
}

static	IMAGE	*
InitSystem(void)
{
	IMAGE	*img;

ENTER_FUNC;
	if		(  ( img = OpenCamera(CamDevice,Width,Height,V4L2_PIX_FMT_RGB24) )  !=  NULL  ) {
		StartCamera(img);
		ThisImage = img;
		dbgprintf("img->h    = %d",img->h);
		dbgprintf("img->w    = %d",img->w);
		dbgprintf("img->size = %d",img->size);
	} else {
		img = NULL;
	}
LEAVE_FUNC;
	return	(img);
}

static	void
StopProcess(
	int		ec)
{
	exit(ec);
}

static int
read_frame(
	FrameArgs	*arg)
{
ENTER_FUNC;
	SleepTimeFiller(arg->filler,1000/FrameRate);
    gdk_draw_rgb_image(arg->area->window,
					   arg->area->style->fg_gc[GTK_STATE_NORMAL], 0, 0,
					   arg->img->w, arg->img->h,
					   GDK_RGB_DITHER_NONE, arg->img->data, arg->img->w * 3);
LEAVE_FUNC;    
	return	(1);
}


extern	int
main(
	int		argc,
	char	*argv[])
{

	FrameArgs	args;
	IMAGE	*img;
	int			status;
    sigset_t sigmask;
	TimeFiller	*filler;
	Bool	fCont;
	CameraConfig	config;

    sigemptyset(&sigmask);
    sigaddset(&sigmask, SIGUSR1);
    sigprocmask(SIG_BLOCK, &sigmask, &SigMask);
#ifdef ENABLE_NLS
	bindtextdomain (GETTEXT_PACKAGE, PACKAGE_LOCALE_DIR);
	bind_textdomain_codeset (GETTEXT_PACKAGE, "UTF-8");
	textdomain (GETTEXT_PACKAGE);
#endif

	(void)signal(SIGPIPE,(void *)StopProcess);
	SetDefault();
	(void)GetOption(option,argc,argv);

	config.brightness = 150;
	config.contrast = 40;
	config.hue = 0;
	config.gamma = 30;
	config.saturation = 20;
	config.line_frequency = 1;
	config.xoff = 0;
	config.yoff = 0;

	para[0].var = &config.brightness;
	para[1].var = &config.contrast;
	para[2].var = &config.hue;
	para[3].var = &config.gamma;
	para[4].var = &config.saturation;
	para[5].var = &config.line_frequency;
	para[6].var = &config.xoff;
	para[7].var = &config.yoff;
	
	GetConfigFile(Config,para);

	gtk_set_locale ();
	gtk_init (&argc, &argv);
	gtk_rc_parse("./gtkrc");
	gdk_rgb_init();

	capture = create_capture ();
	gtk_widget_show (capture);
	img = InitSystem();
	SetCameraConfig(img,&config);

	SetUpWidgetsProperty(capture,img,&config);

	args.filler = InitTimeFiller();
	args.area = lookup_widget(capture,"picture");
	args.img = img;



	gtk_main ();
	FreeTimeFiller(args.filler);
	GetCameraConfig(img,&config);
	PutConfigFile(Config,para);

	StopCamera(img);

    exit (EXIT_SUCCESS);

	return 0;
}

