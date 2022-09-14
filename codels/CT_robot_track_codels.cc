/*
 * Copyright (c) 2019-2021 LAAS/CNRS
 *
 * Authors: Felix Ingrand - LAAS/CNRS and Pascal Chauveau ISAE
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <stdio.h>
#include <err.h>
#include "acCT_robot.h"

#include "CT_robot_c_types.h"

//include for sensor_msgs::Image type
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"

//include for geometry_msgs::Twist type
#include "geometry_msgs/Twist.h"

//for binarisation function:
#include "Detection_codels.hpp"

static const std::string OPENCV_WINDOW = "Image window";

/* Some macros to make my life easier. V2 */
#define check_port_in_p(port) 		(((port)->read(self) == genom_ok) && ((port)->data(self)))
#define check_port_in(port,exception) 	if (!(check_port_in_p(port))) return (exception)(self)
#define check_port_out(port,exception) 	if (!((port)->data(self))) return (exception)(self)

#define bind_port_out_p(port) 		((port ## Data) = (port)->data(self))
#define bind_port_out(port,exception) 	if (!(bind_port_out_p(port))) return (exception)(self)
#define bind_port_in(port,exception) 	if (((port)->read(self) != genom_ok)  || !((port ## Data) = ((port)->data(self)))) return (exception)(self)
#define write_port_p(port) 		((port)->write(self) == genom_ok)
#define write_port(port,exception) 	if (!(write_port_p(port))) return (exception)(self)

// We are not really using ROS so just remap this macro
#define ROS_INFO warnx
#define ROS_ERROR warnx

/* --- Task track ------------------------------------------------------- */


/** Codel InitIDS of task track.
 *
 * Triggered by CT_robot_start.
 * Yields to CT_robot_ether.
 */
genom_event
InitIDS(const CT_robot_CmdPort *CmdPort, CT_robot_cmd_s *cmd,
        int32_t *x, int32_t *y, const genom_context self)
{
  geometry_Twist  *CmdPortData;

  bind_port_out(CmdPort, CT_robot_bad_cmd_port);

  CmdPortData->linear.x = 0.0;
  CmdPortData->linear.y = 0.0;
  CmdPortData->linear.z = 0.0;
  CmdPortData->angular.x = 0.0;
  CmdPortData->angular.y = 0.0;
  CmdPortData->angular.z = 0.0;

  write_port(CmdPort, CT_robot_bad_cmd_port);

  cmd->vx = cmd->wz = 0.0;
  *x = *y = 0;

  return CT_robot_ether;
}


/** Codel CleanIDS of task track.
 *
 * Triggered by CT_robot_stop.
 * Yields to CT_robot_ether.
 */
genom_event
CleanIDS(const CT_robot_CmdPort *CmdPort, const genom_context self)
{
  geometry_Twist  *CmdPortData;

  bind_port_out(CmdPort, CT_robot_bad_cmd_port); 

  CmdPortData->linear.x = 0.0;
  CmdPortData->linear.y = 0.0;
  CmdPortData->linear.z = 0.0;
  CmdPortData->angular.x = 0.0;
  CmdPortData->angular.y = 0.0;
  CmdPortData->angular.z = 0.0;

  write_port(CmdPort, CT_robot_bad_cmd_port); // before quitting... stop the robot.

  return CT_robot_ether;
}


/* --- Activity ColorTrack ---------------------------------------------- */

/** Codel GetImageFindCenter of activity ColorTrack.
 *
 * Triggered by CT_robot_start.
 * Yields to CT_robot_pause_start, CT_robot_CompCmd, CT_robot_Lost,
 *           CT_robot_ether.
 * Throws CT_robot_bad_cmd_port, CT_robot_bad_image_port,
 *        CT_robot_opencv_error.
 */
genom_event
GetImageFindCenter(const CT_robot_ImagePort *ImagePort, int32_t my_r,
                   int32_t my_g, int32_t my_b, int32_t my_seuil,
                   int32_t *x, int32_t *y, int32_t *width,
                   int32_t *height, int32_t verbose,
                   const genom_context self)
{
  static unsigned int seq = 0;
  sensor_Image *ImagePortData;
  sensor_msgs::Image msg;

  bind_port_in(ImagePort,CT_robot_bad_image_port);

  if (seq && (seq == ImagePortData->header.seq)) {
    return CT_robot_pause_start;	// the image is not new, wait the next execution task cycle
  }
  
  // copy ImagePortData in msg...so I can use cv_bridge
  seq = msg.header.seq = ImagePortData->header.seq; 
  msg.header.frame_id = ImagePortData->header.frame_id;
  msg.header.stamp.nsec = ImagePortData->header.stamp.nsec; 
  msg.header.stamp.nsec = ImagePortData->header.stamp.nsec; 
  msg.height = ImagePortData->height; 
  msg.width = ImagePortData->width; 
  msg.encoding = ImagePortData->encoding; 
  msg.is_bigendian = ImagePortData->is_bigendian; 
  msg.step = ImagePortData->step; 

  const int image_size = ImagePortData->height *  ImagePortData->step;
  
  if (image_size != ImagePortData->data._length) {
    warnx("Image wrong size.");
    return CT_robot_bad_image_port(self);
  }

  msg.data.reserve(image_size);
  for( int i=0; i < image_size; i++ ) {
    msg.data.push_back(ImagePortData->data._buffer[i]); 
  }
  
  //necessary for transform ros image type into opencv image type
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    if (verbose) ROS_INFO("I have received an image! ;-)");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return CT_robot_opencv_error(self);
  }
#if CV_VERSION_MAJOR == 4
  IplImage _ipl_img=cvIplImage(cv_ptr->image);
#else
  IplImage _ipl_img=cv_ptr->image;
#endif
  IplImage *ptr_ipl_img= &_ipl_img;

  //declare a CvPoint
  CvPoint coord;

  //call binarisation method!
  coord = binarisation(ptr_ipl_img, my_b, my_g, my_r,my_seuil);


  if (verbose > 0) printf("height: %d,\twidth: %d,\tx: %d,\ty: %d\n",
 			  ptr_ipl_img->height, ptr_ipl_img->width, coord.x, coord.y);

  *x = coord.x;
  *y = coord.y;
  *width = ptr_ipl_img->width;
  *height = ptr_ipl_img->height;
    
  return CT_robot_CompCmd;
}

/** Codel ComputeSpeed of activity ColorTrack.
 *
 * Triggered by CT_robot_CompCmd.
 * Yields to CT_robot_PubCmd.
 * Throws CT_robot_bad_cmd_port, CT_robot_bad_image_port,
 *        CT_robot_opencv_error.
 */
genom_event
ComputeSpeed(int32_t x, int32_t y, int32_t width, int32_t height,
             CT_robot_cmd_s *cmd, int32_t verbose,
             const genom_context self)
{
    float cmd_x_pixel_value= 5.0 / width; // 5 rad/s for the entire width
    float cmd_y_pixel_value= 5.0 / height; // 5 m/s for the entire height

    cmd->wz = - ((x - width/2) * cmd_x_pixel_value);
    cmd->vx = - ((y - height/2) * cmd_y_pixel_value);

    if (verbose > 0) printf("vx: %f,\twz: %f,\txp: %f,\typ: %f\n", 
 			  cmd->vx, cmd->wz, cmd_x_pixel_value, cmd_y_pixel_value);

  return CT_robot_PubCmd;
}

/** Codel PublishSpeed of activity ColorTrack.
 *
 * Triggered by CT_robot_PubCmd.
 * Yields to CT_robot_pause_start, CT_robot_ether.
 * Throws CT_robot_bad_cmd_port, CT_robot_bad_image_port,
 *        CT_robot_opencv_error.
 */
genom_event
PublishSpeed(const CT_robot_cmd_s *cmd,
             const CT_robot_CmdPort *CmdPort,
             const genom_context self)
{
  // This codel publish the ids cmd speed in the port CmdPort.
  geometry_Twist  *CmdPortData;	// A pointer to the CmdPort port data.

  bind_port_out(CmdPort, CT_robot_bad_cmd_port);

  CmdPortData->linear.x = cmd->vx;
  CmdPortData->linear.y = 0.0;
  CmdPortData->linear.z = 0.0;
  CmdPortData->angular.x = 0.0;
  CmdPortData->angular.y = 0.0;
  CmdPortData->angular.z = cmd->wz;

  write_port(CmdPort, CT_robot_bad_cmd_port);

  return CT_robot_pause_start;
}

/** Codel StopRobot of activity ColorTrack.
 *
 * Triggered by CT_robot_stop.
 * Yields to CT_robot_ether.
 * Throws CT_robot_bad_cmd_port, CT_robot_bad_image_port,
 *        CT_robot_opencv_error.
 */
genom_event
StopRobot(CT_robot_cmd_s *cmd, const CT_robot_CmdPort *CmdPort,
          const genom_context self)
{
  // Set the ids cmd and the port CmdPort to zero.
  geometry_Twist  *CmdPortData; 	// A pointer to the CmdPort port data.

  bind_port_out(CmdPort, CT_robot_bad_cmd_port);

  CmdPortData->linear.x = cmd->vx = 0.0;
  CmdPortData->linear.y = 0.0;
  CmdPortData->linear.z = 0.0;
  CmdPortData->angular.x = 0.0;
  CmdPortData->angular.y = 0.0;
  CmdPortData->angular.z = cmd->wz = 0.0;

  write_port(CmdPort, CT_robot_bad_cmd_port);
  
  return CT_robot_ether;
}

/** Codel ComputeWheelWhenLost of activity ColorTrack.
 *
 * Triggered by CT_robot_Lost.
 * Yields to CT_robot_PubCmd.
 * Throws CT_robot_bad_cmd_port, CT_robot_bad_image_port,
 *        CT_robot_opencv_error.
 */
genom_event
ComputeWheelWhenLost(int32_t x, int32_t y, int32_t width,
                     int32_t height, CT_robot_cmd_s *cmd,
                     int32_t verbose, const genom_context self)
{
  if (x == -1)
  {              // We lost it
    cmd->wz = 0.2; // Search the brick
    cmd->vx = 0.2;

    if (verbose > 0)
      printf("Lost the brick; Searching the brick vx: %f,\twz: %f\n",
             cmd->vx, cmd->wz);
  }

  return CT_robot_PubCmd;
}
