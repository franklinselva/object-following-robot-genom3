#include <stdio.h>
#include <err.h>
#include "accnam.h"

#include "cnam_c_types.h"

//add include for manipulate sensor_msgs::Image type:
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"

//TODO add #include for output topic type (geometry_msgs/Twist)
#include "geometry_msgs/Twist.h"

//for binarisation function:
#include "DetectionCnam_codels.hpp"

static const std::string OPENCV_WINDOW = "Image window";

/* Some macros to make my like easier. V2 */
#define check_port_in_p(port) 		(((port)->read(self) == genom_ok) && ((port)->data(self)))
#define check_port_in(port,exception) 	if (!(check_port_in_p(port))) return (exception)(self)
#define check_port_out(port,exception) 	if (!((port)->data(self))) return (exception)(self)

#define bind_port_out_p(port) 		((port ## Data) = (port)->data(self))
#define bind_port_out(port,exception) 	if (!(bind_port_out_p(port))) return (exception)(self)
#define bind_port_in(port,exception) 	if (((port)->read(self) != genom_ok)  || !((port ## Data) = ((port)->data(self)))) return (exception)(self)
#define write_port_p(port) 		((port)->write(self) == genom_ok)
#define write_port(port,exception) 	if (!(write_port_p(port))) return (exception)(self)

#define ROS_INFO warnx
#define ROS_ERROR warnx

/* --- Task track ------------------------------------------------------- */


/** Codel InitIDS of task track.
 *
 * Triggered by cnam_start.
 * Yields to cnam_ether.
 */
genom_event
InitIDS(const cnam_Cmd *Cmd, cnam_cmd_s *cmd, int32_t *x, int32_t *y,
        const genom_context self)
{
  geometry_Twist  *CmdData;

  bind_port_out(Cmd, cnam_bad_cmd_port);

  CmdData->linear.x = 0.0;
  CmdData->linear.y = 0.0;
  CmdData->linear.z = 0.0;
  CmdData->angular.x = 0.0;
  CmdData->angular.y = 0.0;
  CmdData->angular.z = 0.0;

  write_port(Cmd, cnam_bad_cmd_port);

  cmd->vx = cmd->wz = 0.0;
  x = y = 0;

  return cnam_ether;
}


/** Codel CleanIDS of task track.
 *
 * Triggered by cnam_stop.
 * Yields to cnam_ether.
 */
genom_event
CleanIDS(const cnam_Cmd *Cmd, const genom_context self)
{
  geometry_Twist  *CmdData;

  bind_port_out(Cmd, cnam_bad_cmd_port); 

  CmdData->linear.x = 0.0;
  CmdData->linear.y = 0.0;
  CmdData->linear.z = 0.0;
  CmdData->angular.x = 0.0;
  CmdData->angular.y = 0.0;
  CmdData->angular.z = 0.0;

  write_port(Cmd, cnam_bad_cmd_port); // before quitting... stop the robot.

  return cnam_ether;
}


/* --- Activity ColorTrack ---------------------------------------------- */

/** Codel GetImageFindCenter of activity ColorTrack.
 *
 * Triggered by cnam_start.
 * Yields to cnam_pause_start, cnam_CompCmd, cnam_ether.
 * Throws cnam_bad_cmd_port, cnam_bad_image_port, cnam_opencv_error.
 */
genom_event
GetImageFindCenter(const cnam_Image *Image, int32_t my_r, int32_t my_g,
                   int32_t my_b, int32_t my_seuil, int32_t *x,
                   int32_t *y, int32_t *width, int32_t *height,
                   int32_t verbose, const genom_context self)
{
  static unsigned int seq = 0;
  sensor_Image *ImageData;
  sensor_msgs::Image msg;

  bind_port_in(Image,cnam_bad_image_port);

  if (seq && (seq == ImageData->header.seq)) {
    return cnam_pause_start;	// the image is not new, wait next cycle
  }
  
  // copy ImageData in msg...so I can use cv_bridge
  seq = msg.header.seq = ImageData->header.seq; 
  msg.header.frame_id = ImageData->header.frame_id;
  msg.header.stamp.nsec = ImageData->header.stamp.nsec; 
  msg.header.stamp.nsec = ImageData->header.stamp.nsec; 
  msg.height = ImageData->height; 
  msg.width = ImageData->width; 
  msg.encoding = ImageData->encoding; 
  msg.is_bigendian = ImageData->is_bigendian; 
  msg.step = ImageData->step; 

  const int image_size = ImageData->height *  ImageData->step;
  
  if (image_size != ImageData->data._length) {
    warnx("Image wrong size.");
    return cnam_bad_image_port(self);
  }

  msg.data.reserve(image_size);
  for( int i=0; i < image_size; i++ ) {
    msg.data.push_back(ImageData->data._buffer[i]); 
  }
  
  //necessary for transform ros image type into opencv image type
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    if (verbose) ROS_INFO("I have received image! ;-)");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return cnam_opencv_error(self);
  }
  IplImage _ipl_img=cv_ptr->image;
  IplImage *ptr_ipl_img= &_ipl_img;

  //For see OpenCV Image:
  //
  // Update GUI Window
  //  cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  //  cv::waitKey(3);

  //declare a CvPoint
  CvPoint coord;

  //call binarisation method!
  coord = binarisation(ptr_ipl_img, my_b, my_g, my_r,my_seuil);


  if (verbose > 0) printf("height: %d,\twidth: %d,\tx: %d,\ty: %d\n",
 			  ptr_ipl_img->height, ptr_ipl_img->width, coord.x, coord.y);


  // free(msg.header.frame_id);
  // free(msg.encoding);

  *x = coord.x;
  *y = coord.y;
  *width = ptr_ipl_img->width;
  *height = ptr_ipl_img->height;
    
  return cnam_CompCmd;
}

/** Codel ComputeSpeed of activity ColorTrack.
 *
 * Triggered by cnam_CompCmd.
 * Yields to cnam_PubCmd, cnam_ether.
 * Throws cnam_bad_cmd_port, cnam_bad_image_port, cnam_opencv_error.
 */
genom_event
ComputeSpeed(int32_t x, int32_t y, int32_t width, int32_t height,
             cnam_cmd_s *cmd, int32_t verbose,
             const genom_context self)
{
  if (x == -1) {		// We lost it
    cmd->wz = 0;
    cmd->vx = 0;

    if (verbose > 0) printf("Lost the brick vx: %f,\twz: %f\n", 
			    cmd->vx, cmd->wz);
  } else {
    float cibleY = height * 3 / 4;

    float cmd_x_pixel_value= 2.0 / width;
    float cmd_y_pixel_value= 2.0 / (height - cibleY);

    cmd->wz = - ((x - width/2) * cmd_x_pixel_value);
    cmd->vx = - ((y - cibleY) * cmd_y_pixel_value);

    if (verbose > 0) printf("vx: %f,\twz: %f,\txp: %f,\typ: %f\n", 
 			  cmd->vx, cmd->wz, cmd_x_pixel_value, cmd_y_pixel_value);
  }


  return cnam_PubCmd;
}

/** Codel PublishSpeed of activity ColorTrack.
 *
 * Triggered by cnam_PubCmd.
 * Yields to cnam_pause_start, cnam_ether.
 * Throws cnam_bad_cmd_port, cnam_bad_image_port, cnam_opencv_error.
 */
genom_event
PublishSpeed(const cnam_cmd_s *cmd, const cnam_Cmd *Cmd,
             const genom_context self)
{
  geometry_Twist  *CmdData;

  bind_port_out(Cmd, cnam_bad_cmd_port);

  CmdData->linear.x = cmd->vx;
  CmdData->linear.y = 0.0;
  CmdData->linear.z = 0.0;
  CmdData->angular.x = 0.0;
  CmdData->angular.y = 0.0;
  CmdData->angular.z = cmd->wz;

  write_port(Cmd, cnam_bad_cmd_port);

  return cnam_pause_start;
}

/** Codel StopRobot of activity ColorTrack.
 *
 * Triggered by cnam_stop.
 * Yields to cnam_ether.
 * Throws cnam_bad_cmd_port, cnam_bad_image_port, cnam_opencv_error.
 */
genom_event
StopRobot(cnam_cmd_s *cmd, const cnam_Cmd *Cmd,
          const genom_context self)
{
  geometry_Twist  *CmdData;

  bind_port_out(Cmd, cnam_bad_cmd_port);

  CmdData->linear.x = cmd->vx = 0.0;
  CmdData->linear.y = 0.0;
  CmdData->linear.z = 0.0;
  CmdData->angular.x = 0.0;
  CmdData->angular.y = 0.0;
  CmdData->angular.z = cmd->wz = 0.0;

  write_port(Cmd, cnam_bad_cmd_port);
  
  return cnam_ether;
}
