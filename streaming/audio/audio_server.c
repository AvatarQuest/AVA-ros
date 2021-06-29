#include <gst/gst.h>
#include <string.h>
/*
 AVATAR CLIENT: gst-launch-1.0 tcpclientsrc port=8081 host=192.168.0.114 ! rawaudioparse ! audioconvert ! audioresample ! alsasink
 AVATAR SERVER: gst-launch-1.0 alsasrc ! queue ! audio/x-raw,rate=44100,channels=2 ! tcpserversink port=8080 host=0.0.0.0
 OP CLIENT: gst-launch-1.0 tcpclientsrc port=8080 host=192.168.0.137 ! rawaudioparse ! audioconvert ! audioresample ! autoaudiosink
 OP SERVER: ./gst-launch-1.0 directsoundsrc ! queue ! audio/x-raw ! tcpserversink port=8081 host=0.0.0.0
 */
// gst-launch-1.0 alsasrc ! queue ! audio/x-raw,rate=44100,channels=2 ! tcpserversink port=8080 host=0.0.0.0
// gst-launch-1.0 -v alsasrc ! queue ! audio/x-raw,rate=44100,channels=2 ! lamemp3enc target=bitrate cbr=true bitrate=120 ! filesink location=test.mp3

const int rate = 44100;
const int channels = 2;
const int port = 8080;



int main(int argc, char *argv[]) {
  GstElement *pipeline, *scarlett_src, *queue, *raw_parser, *tcpserver;
  GstBus *bus;
  GstStateChangeReturn ret;
  GMainLoop *main_loop;
  CustomData data;

  /* Initialize GStreamer */
  gst_init (&argc, &argv);

  /* Initialize our data structure */
  scarlett_src = gst_element_factory_make("alsasrc", "scarlett_src");
  queue = gst_element_factory_make("queue", "queue");
  raw_parser = gst_element_factory_make("")

  /* Build the pipeline */
  

  /* Start playing */
  ret = gst_element_set_state (pipeline, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    g_printerr ("Unable to set the pipeline to the playing state.\n");
    gst_object_unref (pipeline);
    return -1;
  } else if (ret == GST_STATE_CHANGE_NO_PREROLL) {
    data.is_live = TRUE;
  }

  main_loop = g_main_loop_new (NULL, FALSE);
  data.loop = main_loop;
  data.pipeline = pipeline;

  gst_bus_add_signal_watch (bus);
  g_signal_connect (bus, "message", G_CALLBACK (cb_message), &data);

  g_main_loop_run (main_loop);

  /* Free resources */
  g_main_loop_unref (main_loop);
  gst_object_unref (bus);
  gst_element_set_state (pipeline, GST_STATE_NULL);
  gst_object_unref (pipeline);
  return 0;
}
