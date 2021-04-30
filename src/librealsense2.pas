(* License: Apache 2.0. See LICENSE file in root directory.
   Copyright(c) 2017 Intel Corporation. All Rights Reserved. *)

(** \file rs.h
 * \brief
 * Exposes librealsense functionality for C compilers
 *)

(** \file rs_types.h
 * \brief
 * Exposes RealSense structs
 *)

(*** \file rs_context.h
 * \brief Exposes RealSense context functionality for C compilers
 *)

(*** \file rs_device.h
 * \brief Exposes RealSense device functionality for C compilers
 *)

(*** \file rs_frame.h
 * \brief
 * Exposes RealSense frame functionality for C compilers
 *)

(*** \file rs_option.h
 * \brief
 * Exposes sensor options functionality for C compilers
 *)

(*** \file rs_processing.h
 * \brief
 * Exposes RealSense processing-block functionality for C compilers
 *)

(*** \file rs_record_playback.h
 * \brief
 * Exposes record and playback functionality for C compilers
 *)

(*** \file rs_sensor.h
 * \brief
 * Exposes RealSense sensor functionality for C compilers
 *)

(** \file rs_pipeline.h
 * \brief
 * Exposes RealSense processing-block functionality for C compilers
 *)

unit librealsense2;

{$MODE delphi}
{$INCLUDE 'librealsense2.inc'}

interface

uses
{$IFDEF WINDOWS}
  Windows,
{$ENDIF}
  SysUtils;

{$IFDEF WINDOWS}
const
  LibRealsense2_Name = 'realsense2.dll';
{$ENDIF}
{$IFDEF LINUX}
const
  LibRealsense2_Name = 'realsense2.so';
{$ENDIF}
{$IFDEF DARWIN}
const
  {$linkframework librealsense2}
{$ENDIF}


//------------------------------------------------------------------------------
type
(** \brief Category of the librealsense notification. *)
Trs2_notification_category = (
    RS2_NOTIFICATION_CATEGORY_FRAMES_TIMEOUT,           (**< Frames didn't arrived within 5 seconds *)
    RS2_NOTIFICATION_CATEGORY_FRAME_CORRUPTED,          (**< Received partial/incomplete frame *)
    RS2_NOTIFICATION_CATEGORY_HARDWARE_ERROR,           (**< Error reported from the device *)
    RS2_NOTIFICATION_CATEGORY_HARDWARE_EVENT,           (**< General Hardeware notification that is not an error *)
    RS2_NOTIFICATION_CATEGORY_UNKNOWN_ERROR,            (**< Received unknown error from the device *)
    RS2_NOTIFICATION_CATEGORY_FIRMWARE_UPDATE_RECOMMENDED,  (**< Current firmware version installed is not the latest available *)
    RS2_NOTIFICATION_CATEGORY_POSE_RELOCALIZATION,      (**< A relocalization event has updated the pose provided by a pose sensor *)
    RS2_NOTIFICATION_CATEGORY_COUNT                     (**< Number of enumeration values. Not a valid input: intended to be used in for-loops. *)
);

Trs2_notification_category_to_string = function (category: Trs2_notification_category): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

type
(** \brief Exception types are the different categories of errors that RealSense API might return. *)
Trs2_exception_type = (
    RS2_EXCEPTION_TYPE_UNKNOWN,
    RS2_EXCEPTION_TYPE_CAMERA_DISCONNECTED,  (**< Device was disconnected, this can be caused by outside intervention, by internal firmware error or due to insufficient power *)
    RS2_EXCEPTION_TYPE_BACKEND,              (**< Error was returned from the underlying OS-specific layer *)
    RS2_EXCEPTION_TYPE_INVALID_VALUE,        (**< Invalid value was passed to the API *)
    RS2_EXCEPTION_TYPE_WRONG_API_CALL_SEQUENCE,  (**< Function precondition was violated *)
    RS2_EXCEPTION_TYPE_NOT_IMPLEMENTED,      (**< The method is not implemented at this point *)
    RS2_EXCEPTION_TYPE_DEVICE_IN_RECOVERY_MODE,  (**< Device is in recovery mode and might require firmware update *)
    RS2_EXCEPTION_TYPE_IO,                   (**< IO Device failure *)
    RS2_EXCEPTION_TYPE_COUNT                 (**< Number of enumeration values. Not a valid input: intended to be used in for-loops. *)
);
Trs2_exception_type_to_string = function(_type: Trs2_exception_type): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

type
(** \brief Distortion model: defines how pixel coordinates should be mapped to sensor coordinates. *)
Trs2_distortion = (
    RS2_DISTORTION_NONE                  , (**< Rectilinear images. No distortion compensation required. *)
    RS2_DISTORTION_MODIFIED_BROWN_CONRADY, (**< Equivalent to Brown-Conrady distortion, except that tangential distortion is applied to radially distorted points *)
    RS2_DISTORTION_INVERSE_BROWN_CONRADY , (**< Equivalent to Brown-Conrady distortion, except undistorts image instead of distorting it *)
    RS2_DISTORTION_FTHETA                , (**< F-Theta fish-eye distortion model *)
    RS2_DISTORTION_BROWN_CONRADY         , (**< Unmodified Brown-Conrady distortion model *)
    RS2_DISTORTION_KANNALA_BRANDT4       , (**< Four parameter Kannala Brandt distortion model *)
    RS2_DISTORTION_COUNT               (**< Number of enumeration values. Not a valid input: intended to be used in for-loops. *)
);

Trs2_distortion_to_string = function(distortion: Trs2_distortion): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

type
(** \brief Video stream intrinsics. *)
Prs2_intrinsics = ^Trs2_intrinsics;
Trs2_intrinsics = record
  width: Integer;              (**< Width of the image in pixels *)
  height: Integer;             (**< Height of the image in pixels *)
  ppx: Single;                 (**< Horizontal coordinate of the principal point of the image, as a pixel offset from the left edge *)
  ppy: Single;                 (**< Vertical coordinate of the principal point of the image, as a pixel offset from the top edge *)
  fx: Single;                  (**< Focal length of the image plane, as a multiple of pixel width *)
  fy: SIngle;                  (**< Focal length of the image plane, as a multiple of pixel height *)
  model: Trs2_distortion;      (**< Distortion model of the image *)
  coeffs: Array [0..4] of Single;  (**< Distortion coefficients *)
end;

(** \brief Video DSM (Digital Sync Module) parameters for calibration (same layout as in FW ac_depth_params)
    This is the block in MC that converts angles to dimensionless integers reported to MA (using "DSM coefficients").
*)
Prs2_dsm_params = ^Trs2_dsm_params;
Trs2_dsm_params = record
  timestamp: UInt64;            (***< system_clock::time_point::time_since_epoch().count() *)
  version: Word;                (***< MAJOR<<12 | MINOR<<4 | PATCH *)
  model: Byte;                  (***< rs2_dsm_correction_model *)
  flags: Array [0..4] of Byte;  (***< TBD, now 0s *)
  h_scale: Single;              (***< the scale factor to horizontal DSM scale thermal results *)
  v_scale: Single;              (***< the scale factor to vertical DSM scale thermal results *)
  h_offset: Single;             (***< the offset to horizontal DSM offset thermal results *)
  v_offset: Single;             (***< the offset to vertical DSM offset thermal results *)
  rtd_offset: Single;           (***< the offset to the Round-Trip-Distance delay thermal results *)
  temp_x2: Byte;                (***< the temperature recorded times 2 (ldd for depth; hum for rgb) *)
  reserved: Array [0..10] of Byte;
end;

Trs2_dsm_correction_model = (
    RS2_DSM_CORRECTION_NONE,    (***< hFactor and hOffset are not used, and no artificial error is induced *)
    RS2_DSM_CORRECTION_AOT,     (***< Aging-over-thermal (default); aging-induced error is uniform across temperature *)
    RS2_DSM_CORRECTION_TOA,     (***< Thermal-over-aging; aging-induced error changes alongside temperature *)
    RS2_DSM_CORRECTION_COUNT
);

(*** \brief Motion device intrinsics: scale, bias, and variances. *)
Prs2_motion_device_intrinsic = ^Trs2_motion_device_intrinsic;
Trs2_motion_device_intrinsic = record
(** \internal
    * Scale X       cross axis  cross axis  Bias X \n
    * cross axis    Scale Y     cross axis  Bias Y \n
    * cross axis    cross axis  Scale Z     Bias Z *)
  data: Array [0..2]of Array [0..3] of Single; (***< Interpret data array values *)

  noise_variances: Array [0..2] of Single; (***< Variance of noise for X, Y, and Z axis *)
  bias_variances: Array [0..2] of Single ; (***< Variance of bias for X, Y, and Z axis *)
end;

(*** \brief 3D coordinates with origin at topmost left corner of the lense,
     with positive Z pointing away from the camera, positive X pointing camera right and positive Y pointing camera down *)
Prs2_vertex = ^Trs2_vertex;
Trs2_vertex = record
  xyz: Array [0..2] of Single;
end;

(*** \brief Pixel location within 2D image. (0,0) is the topmost, left corner. Positive X is right, positive Y is down *)
Prs2_pixel = ^Trs2_pixel;
Trs2_pixel = record
  ij: Array [0..1] of Integer;
end;

(*** \brief 3D vector in Euclidean coordinate space *)
Prs2_vector = ^Trs2_vector;
Trs2_vector = record
  x, y, z: Single;
end;

(*** \brief Quaternion used to represent rotation  *)
Prs2_quaternion = ^Trs2_quaternion;
Trs2_quaternion = record
  x, y, z, w: Single;
end;

Prs2_pose = ^Trs2_pose;
Trs2_pose = record
  translation: Trs2_vector;      (***< X, Y, Z values of translation, in meters (relative to initial position)                                   *)
  velocity: Trs2_vector;         (***< X, Y, Z values of velocity, in meters/sec                                                                 *)
  acceleration: Trs2_vector;     (***< X, Y, Z values of acceleration, in meters/sec^2                                                           *)
  rotation: Trs2_quaternion;     (***< Qi, Qj, Qk, Qr components of rotation as represented in quaternion rotation (relative to initial position) *)
  angular_velocity: Trs2_vector; (***< X, Y, Z values of angular velocity, in radians/sec                                                        *)
  angular_acceleration: Trs2_vector; (***< X, Y, Z values of angular acceleration, in radians/sec^2                                                  *)
  tracker_confidence: FixedUInt; (***< Pose confidence 0x0 - Failed, 0x1 - Low, 0x2 - Medium, 0x3 - High                                         *)
  mapper_confidence: FixedUInt;  (***< Pose map confidence 0x0 - Failed, 0x1 - Low, 0x2 - Medium, 0x3 - High                                     *)
end;

(*** \brief Severity of the librealsense logger. *)
Trs2_log_severity = (
    RS2_LOG_SEVERITY_DEBUG, (***< Detailed information about ordinary operations *)
    RS2_LOG_SEVERITY_INFO , (***< Terse information about ordinary operations *)
    RS2_LOG_SEVERITY_WARN , (***< Indication of possible failure *)
    RS2_LOG_SEVERITY_ERROR, (***< Indication of definite failure *)
    RS2_LOG_SEVERITY_FATAL, (***< Indication of unrecoverable failure *)
    RS2_LOG_SEVERITY_NONE , (***< No logging will occur *)
    RS2_LOG_SEVERITY_COUNT, (***< Number of enumeration values. Not a valid input: intended to be used in for-loops. *)
    RS2_LOG_SEVERITY_ALL = RS2_LOG_SEVERITY_DEBUG   (***< Include any/all log messages *)
);
Trs2_log_severity_to_string = function(info: Trs2_log_severity): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

type

(*** \brief Specifies advanced interfaces (capabilities) objects may implement. *)
Trs2_extension = (
    RS2_EXTENSION_UNKNOWN,
    RS2_EXTENSION_DEBUG,
    RS2_EXTENSION_INFO,
    RS2_EXTENSION_MOTION,
    RS2_EXTENSION_OPTIONS,
    RS2_EXTENSION_VIDEO,
    RS2_EXTENSION_ROI,
    RS2_EXTENSION_DEPTH_SENSOR,
    RS2_EXTENSION_VIDEO_FRAME,
    RS2_EXTENSION_MOTION_FRAME,
    RS2_EXTENSION_COMPOSITE_FRAME,
    RS2_EXTENSION_POINTS,
    RS2_EXTENSION_DEPTH_FRAME,
    RS2_EXTENSION_ADVANCED_MODE,
    RS2_EXTENSION_RECORD,
    RS2_EXTENSION_VIDEO_PROFILE,
    RS2_EXTENSION_PLAYBACK,
    RS2_EXTENSION_DEPTH_STEREO_SENSOR,
    RS2_EXTENSION_DISPARITY_FRAME,
    RS2_EXTENSION_MOTION_PROFILE,
    RS2_EXTENSION_POSE_FRAME,
    RS2_EXTENSION_POSE_PROFILE,
    RS2_EXTENSION_TM2,
    RS2_EXTENSION_SOFTWARE_DEVICE,
    RS2_EXTENSION_SOFTWARE_SENSOR,
    RS2_EXTENSION_DECIMATION_FILTER,
    RS2_EXTENSION_THRESHOLD_FILTER,
    RS2_EXTENSION_DISPARITY_FILTER,
    RS2_EXTENSION_SPATIAL_FILTER,
    RS2_EXTENSION_TEMPORAL_FILTER,
    RS2_EXTENSION_HOLE_FILLING_FILTER,
    RS2_EXTENSION_ZERO_ORDER_FILTER,
    RS2_EXTENSION_RECOMMENDED_FILTERS,
    RS2_EXTENSION_POSE,
    RS2_EXTENSION_POSE_SENSOR,
    RS2_EXTENSION_WHEEL_ODOMETER,
    RS2_EXTENSION_GLOBAL_TIMER,
    RS2_EXTENSION_UPDATABLE,
    RS2_EXTENSION_UPDATE_DEVICE,
    RS2_EXTENSION_L500_DEPTH_SENSOR,
    RS2_EXTENSION_TM2_SENSOR,
    RS2_EXTENSION_AUTO_CALIBRATED_DEVICE,
    RS2_EXTENSION_COLOR_SENSOR,
    RS2_EXTENSION_MOTION_SENSOR,
    RS2_EXTENSION_FISHEYE_SENSOR,
    RS2_EXTENSION_DEPTH_HUFFMAN_DECODER,
    RS2_EXTENSION_SERIALIZABLE,
    RS2_EXTENSION_FW_LOGGER,
    RS2_EXTENSION_AUTO_CALIBRATION_FILTER,
    RS2_EXTENSION_DEVICE_CALIBRATION,
    RS2_EXTENSION_CALIBRATED_SENSOR,
    RS2_EXTENSION_HDR_MERGE,
    RS2_EXTENSION_SEQUENCE_ID_FILTER,
    RS2_EXTENSION_COUNT
);
Trs2_extension_type_to_string = function(type_: Trs2_extension): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};
Trs2_extension_to_string = function(type_: Trs2_extension): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

type
(*** \brief Specifies types of different matchers *)
Trs2_matchers = (
   RS2_MATCHER_DI,      //compare depth and ir based on frame number

   RS2_MATCHER_DI_C,    //compare depth and ir based on frame number,
                        //compare the pair of corresponding depth and ir with color based on closest timestamp,
                        //commonly used by SR300

   RS2_MATCHER_DLR_C,   //compare depth, left and right ir based on frame number,
                        //compare the set of corresponding depth, left and right with color based on closest timestamp,
                        //commonly used by RS415, RS435

   RS2_MATCHER_DLR,     //compare depth, left and right ir based on frame number,
                        //commonly used by RS400, RS405, RS410, RS420, RS430

   RS2_MATCHER_DIC,     //compare depth, ir and confidence based on frame number used by RS500

   RS2_MATCHER_DIC_C,    //compare depth, ir and confidence based on frame number,
                         //compare the set of corresponding depth, ir and confidence with color based on closest timestamp,
                         //commonly used by RS515

   RS2_MATCHER_DEFAULT, //the default matcher compare all the streams based on closest timestamp

   RS2_MATCHER_COUNT
);

type
  PPrs2_device_info                      = ^Prs2_device_info;
  Prs2_device_info                       = ^Trs2_device_info;
  Trs2_device_info                       = Pointer;

  PPrs2_device                           = ^Prs2_device;
  Prs2_device                            = ^Trs2_device;
  Trs2_device                            = Pointer;

  PPrs2_error                            = ^Prs2_error;
  Prs2_error                             = ^Trs2_error;
  Trs2_error                             = Pointer;

  PPrs2_log_message                      = ^Prs2_log_message;
  Prs2_log_message                       = ^Trs2_log_message;
  Trs2_log_message                       = Pointer;

  PPrs2_raw_data_buffer                  = ^Prs2_raw_data_buffer;
  Prs2_raw_data_buffer                   = ^Trs2_raw_data_buffer;
  Trs2_raw_data_buffer                   = Pointer;

  PPrs2_frame                            = ^Prs2_frame;
  Prs2_frame                             = ^Trs2_frame;
  Trs2_frame                             = Pointer;

  PPrs2_frame_queue                      = ^Prs2_frame_queue;
  Prs2_frame_queue                       = ^Trs2_frame_queue;
  Trs2_frame_queue                       = Pointer;

  PPrs2_pipeline                         = ^Prs2_pipeline;
  Prs2_pipeline                          = ^Trs2_pipeline;
  Trs2_pipeline                          = Pointer;

  PPrs2_pipeline_profile                 = ^Prs2_pipeline_profile;
  Prs2_pipeline_profile                  = ^Trs2_pipeline_profile;
  Trs2_pipeline_profile                  = Pointer;

  PPrs2_config                           = ^Prs2_config;
  Prs2_config                            = ^Trs2_config;
  Trs2_config                            = Pointer;

  PPrs2_device_list                      = ^Prs2_device_list;
  Prs2_device_list                       = ^Trs2_device_list;
  Trs2_device_list                       = Pointer;

  PPrs2_stream_profile_list              = ^Prs2_stream_profile_list;
  Prs2_stream_profile_list               = ^Trs2_stream_profile_list;
  Trs2_stream_profile_list               = Pointer;

  PPrs2_processing_block_list            = ^Prs2_processing_block_list;
  Prs2_processing_block_list             = ^Trs2_processing_block_list;
  Trs2_processing_block_list             = Pointer;

  PPrs2_stream_profile                   = ^Prs2_stream_profile;
  Prs2_stream_profile                    = ^Trs2_stream_profile;
  Trs2_stream_profile                    = Pointer;

  PPrs2_frame_callback                   = ^Prs2_frame_callback;
  Prs2_frame_callback                    = ^Trs2_frame_callback;
  Trs2_frame_callback                    = Pointer;

  PPrs2_log_callback                     =^Prs2_log_callback;
  Prs2_log_callback                      = ^Trs2_log_callback;
  Trs2_log_callback                      = Pointer;

  PPrs2_syncer                           = ^Prs2_syncer;
  Prs2_syncer                            = ^Trs2_syncer;
  Trs2_syncer                            = Pointer;

  PPrs2_device_serializer                = ^Prs2_device_serializer;
  Prs2_device_serializer                 = ^Trs2_device_serializer;
  Trs2_device_serializer                 = Pointer;

  PPrs2_source                           = ^Prs2_source;
  Prs2_source                            = ^Trs2_source;
  Trs2_source                            = Pointer;

  PPrs2_processing_block                 = ^Prs2_processing_block;
  Prs2_processing_block                  = ^Trs2_processing_block;
  Trs2_processing_block                  = Pointer;

  PPrs2_frame_processor_callback         = ^Prs2_frame_processor_callback;
  Prs2_frame_processor_callback          = ^Trs2_frame_processor_callback;
  Trs2_frame_processor_callback          = Pointer;

  PPrs2_playback_status_changed_callback = ^Prs2_playback_status_changed_callback;
  Prs2_playback_status_changed_callback  = ^Trs2_playback_status_changed_callback;
  Trs2_playback_status_changed_callback  = Pointer;

  PPrs2_update_progress_callback         = ^Prs2_update_progress_callback;
  Prs2_update_progress_callback          = ^Trs2_update_progress_callback;
  Trs2_update_progress_callback          = Pointer;

  PPrs2_context                          = ^Prs2_context;
  Prs2_context                           = ^Trs2_context;
  Trs2_context                           = Pointer;

  PPrs2_device_hub                       = ^Prs2_device_hub;
  Prs2_device_hub                        = ^Trs2_device_hub;
  Trs2_device_hub                        = Pointer;

  PPrs2_sensor_list                      = ^Prs2_sensor_list;
  Prs2_sensor_list                       = ^Trs2_sensor_list;
  Trs2_sensor_list                       = Pointer;

  PPrs2_sensor                           = ^Prs2_sensor;
  Prs2_sensor                            = ^Trs2_sensor;
  Trs2_sensor                            = Pointer;

  PPrs2_options                          = ^Prs2_options;
  Prs2_options                           = ^Trs2_options;
  Trs2_options                           = Pointer;

  PPrs2_options_list                     = ^Prs2_options_list;
  Prs2_options_list                      = ^Trs2_options_list;
  Trs2_options_list                      = Pointer;

  PPrs2_devices_changed_callback         = ^Prs2_devices_changed_callback;
  Prs2_devices_changed_callback          = ^Trs2_devices_changed_callback;
  Trs2_devices_changed_callback          = Pointer;

  PPrs2_notification                     = ^Prs2_notification;
  Prs2_notification                      = ^Trs2_notification;
  Trs2_notification                      = Pointer;

  PPrs2_notifications_callback           = ^Prs2_notifications_callback;
  Prs2_notifications_callback            = ^Trs2_notifications_callback;
  Trs2_notifications_callback            = Pointer;

  PPrs2_firmware_log_message             = ^Prs2_firmware_log_message;
  Prs2_firmware_log_message              = ^Trs2_firmware_log_message;
  Trs2_firmware_log_message              = Pointer;

  PPrs2_firmware_log_parsed_message      = ^Prs2_firmware_log_parsed_message;
  Prs2_firmware_log_parsed_message       = ^Trs2_firmware_log_parsed_message;
  Trs2_firmware_log_parsed_message       = Pointer;

  PPrs2_firmware_log_parser              = ^Prs2_firmware_log_parser;
  Prs2_firmware_log_parser               = ^Trs2_firmware_log_parser;
  Trs2_firmware_log_parser               = Pointer;

  PPrs2_terminal_parser                  = ^Prs2_terminal_parser;
  Prs2_terminal_parser                   = ^Trs2_terminal_parser;
  Trs2_terminal_parser                   = Pointer;
//typedef void (*rs2_log_callback_ptr)(rs2_log_severity, rs2_log_message const *, void * arg);
//typedef void (*rs2_notification_callback_ptr)(rs2_notification*, void*);
//typedef void(*rs2_software_device_destruction_callback_ptr)(void*);
//typedef void (*rs2_devices_changed_callback_ptr)(rs2_device_list*, rs2_device_list*, void*);
//typedef void (*rs2_frame_callback_ptr)(rs2_frame*, void*);
//typedef void (*rs2_frame_processor_callback_ptr)(rs2_frame*, rs2_source*, void*);
//typedef void(*rs2_update_progress_callback_ptr)(const float, void*);

  PPrs2_time = ^Prs2_time;
  Prs2_time = ^Trs2_time;
  Trs2_time = Double; (***< Timestamp format. units are milliseconds *)

  PPrs2_metadata_type = ^Prs2_metadata_type;
  Prs2_metadata_type = ^Trs2_metadata_type;
  Trs2_metadata_type = Int64; (***< Metadata attribute type is defined as 64 bit signed integer*)

Trs2_create_error = function(const what: PAnsiChar;
                             const name: PAnsiChar;
                             const args: PAnsiChar;
                             type_: Trs2_exception_type): Prs2_error; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

Trs2_get_librealsense_exception_type = function(const error: Prs2_error): Trs2_exception_type; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};
Trs2_get_failed_function = function(const error: Prs2_error): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};
Trs2_get_failed_args = function(const error: Prs2_error): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};
Trs2_get_error_message = function(const error: Prs2_error): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};
Trs2_free_error = procedure(const error: Prs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

//------------------------------------------------------------------------------

(***
 * \brief Creates RealSense context that is required for the rest of the API.
 * \param[in] api_version Users are expected to pass their version of \c RS2_API_VERSION to make sure they are running the correct librealsense version.
 * \param[out] error  If non-null, receives any error that occurs during this call, otherwise, errors are ignored.
 * \return            Context object
 *)
Trs2_create_context = function(api_version: Integer;
                               error: PPrs2_error): Prs2_context; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * \brief Frees the relevant context object.
 * \param[in] context Object that is no longer needed
 *)
Trs2_delete_context = procedure(context: Prs2_context); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * set callback to get devices changed events
 * these events will be raised by the context whenever new RealSense device is connected or existing device gets disconnected
 * \param context     Object representing librealsense session
 * \param[in] callback callback object created from c++ application. ownership over the callback object is moved into the context
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_set_devices_changed_callback_cpp = procedure(context: Prs2_context;
                                                  callback: Prs2_devices_changed_callback;
                                                  error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * set callback to get devices changed events
 * these events will be raised by the context whenever new RealSense device is connected or existing device gets disconnected
 * \param context     Object representing librealsense session
 * \param[in] callback function pointer to register as per-notifications callback
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_set_devices_changed_callback = procedure(const context: Prs2_context;
                                              //callback: Prs2_devices_changed_callback_ptr;
                                              user: Pointer;
                                              error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Create a new device and add it to the context
 * \param ctx   The context to which the new device will be added
 * \param file  The file from which the device should be created
 * \param[out] error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * @return  A pointer to a device that plays data from the file, or null in case of failure
 *)
Trs2_context_add_device = function(ctx: Prs2_context;
                                   const _file: PAnsiChar;
                                   error: PPrs2_error): Prs2_device; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Add an instance of software device to the context
 * \param ctx   The context to which the new device will be added
 * \param dev   Instance of software device to register into the context
 * \param[out] error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_context_add_software_device = procedure(ctx: Prs2_context;
                                             dev: Prs2_device;
                                             error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Removes a playback device from the context, if exists
 * \param[in]  ctx       The context from which the device should be removed
 * \param[in]  file      The file name that was used to add the device
 * \param[out] error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_context_remove_device = procedure(ctx: Prs2_context;
                                       const _file: PAnsiChar;
                                       error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Removes tracking module.
 * function query_devices() locks the tracking module in the tm_context object.
 * If the tracking module device is not used it should be removed using this function, so that other applications could find it.
 * This function can be used both before the call to query_device() to prevent enabling tracking modules or afterwards to
 * release them.
 *)
Trs2_context_unload_tracking_module = procedure(ctx: Prs2_context;
                                                error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * create a static snapshot of all connected devices at the time of the call
 * \param context     Object representing librealsense session
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return            the list of devices, should be released by rs2_delete_device_list
 *)
Trs2_query_devices = function(const context: Prs2_context;
                              error: PPrs2_error): Prs2_device_list; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

const
  RS2_PRODUCT_LINE_ANY            = $ff;
  RS2_PRODUCT_LINE_ANY_INTEL      = $fe;
  RS2_PRODUCT_LINE_NON_INTEL      = $01;
  RS2_PRODUCT_LINE_D400           = $02;
  RS2_PRODUCT_LINE_SR300          = $04;
  RS2_PRODUCT_LINE_L500           = $08;
  RS2_PRODUCT_LINE_T200           = $10;
  RS2_PRODUCT_LINE_DEPTH          = (RS2_PRODUCT_LINE_L500 or RS2_PRODUCT_LINE_SR300 or RS2_PRODUCT_LINE_D400);
  RS2_PRODUCT_LINE_TRACKING       = RS2_PRODUCT_LINE_T200;

type
(***
 * create a static snapshot of all connected devices at the time of the call
 * \param context     Object representing librealsense session
 * \param product_mask Controls what kind of devices will be returned
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return            the list of devices, should be released by rs2_delete_device_list
 *)
Trs2_query_devices_ex = function(const context: Prs2_context;
                                 product_mask: Integer;
                                 error: PPrs2_error): Prs2_device_list; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * \brief Creates RealSense device_hub .
 * \param[in] context The context for the device hub
 * \param[out] error  If non-null, receives any error that occurs during this call, otherwise, errors are ignored.
 * \return            Device hub object
 *)
Trs2_create_device_hub = function(const context: Prs2_context;
                                  error: PPrs2_error): Prs2_device_hub; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * \brief Frees the relevant device hub object.
 * \param[in] hub Object that is no longer needed
 *)
Trs2_delete_device_hub = procedure(const hub: Prs2_device_hub); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * If any device is connected return it, otherwise wait until next RealSense device connects.
 * Calling this method multiple times will cycle through connected devices
 * \param[in] ctx The context to creat the device
 * \param[in] hub The device hub object
 * \param[out] error  If non-null, receives any error that occurs during this call, otherwise, errors are ignored.
 * \return            device object
 *)
Trs2_device_hub_wait_for_device = function(const hub: Prs2_device_hub;
                                           error: PPrs2_error): Prs2_device; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Checks if device is still connected
 * \param[in] hub The device hub object
 * \param[in] device The device
 * \param[out] error  If non-null, receives any error that occurs during this call, otherwise, errors are ignored.
 * \return            1 if the device is connected, 0 otherwise
 *)
Trs2_device_hub_is_device_connected = function(const hub: Prs2_device_hub;
                                               const device: Prs2_device;
                                               error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};


//------------------------------------------------------------------------------


type
(*** \brief Read-only strings that can be queried from the device.
   Not all information attributes are available on all camera types.
   This information is mainly available for camera debug and troubleshooting and should not be used in applications. *)
Trs2_camera_info = (
  RS2_CAMERA_INFO_NAME                           , (***< Friendly name *)
  RS2_CAMERA_INFO_SERIAL_NUMBER                  , (***< Device serial number *)
  RS2_CAMERA_INFO_FIRMWARE_VERSION               , (***< Primary firmware version *)
  RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION   , (***< Recommended firmware version *)
  RS2_CAMERA_INFO_PHYSICAL_PORT                  , (***< Unique identifier of the port the device is connected to (platform specific) *)
  RS2_CAMERA_INFO_DEBUG_OP_CODE                  , (***< If device supports firmware logging, this is the command to send to get logs from firmware *)
  RS2_CAMERA_INFO_ADVANCED_MODE                  , (***< True iff the device is in advanced mode *)
  RS2_CAMERA_INFO_PRODUCT_ID                     , (***< Product ID as reported in the USB descriptor *)
  RS2_CAMERA_INFO_CAMERA_LOCKED                  , (***< True iff EEPROM is locked *)
  RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR            , (***< Designated USB specification: USB2/USB3 *)
  RS2_CAMERA_INFO_PRODUCT_LINE                   , (***< Device product line D400/SR300/L500/T200 *)
  RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER             , (***< ASIC serial number *)
  RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID             , (***< Firmware update ID *)
  RS2_CAMERA_INFO_IP_ADDRESS                     , (***< IP address for remote camera. *)
  RS2_CAMERA_INFO_COUNT                        (***< Number of enumeration values. Not a valid input: intended to be used in for-loops. *)
);
Trs2_camera_info_to_string = function(info: Trs2_camera_info): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

type
(*** \brief Streams are different types of data provided by RealSense devices. *)

Prs2_stream = ^Trs2_stream;
Trs2_stream = (
  RS2_STREAM_ANY,
  RS2_STREAM_DEPTH                            , (***< Native stream of depth data produced by RealSense device *)
  RS2_STREAM_COLOR                            , (***< Native stream of color data captured by RealSense device *)
  RS2_STREAM_INFRARED                         , (***< Native stream of infrared data captured by RealSense device *)
  RS2_STREAM_FISHEYE                          , (***< Native stream of fish-eye (wide) data captured from the dedicate motion camera *)
  RS2_STREAM_GYRO                             , (***< Native stream of gyroscope motion data produced by RealSense device *)
  RS2_STREAM_ACCEL                            , (***< Native stream of accelerometer motion data produced by RealSense device *)
  RS2_STREAM_GPIO                             , (***< Signals from external device connected through GPIO *)
  RS2_STREAM_POSE                             , (***< 6 Degrees of Freedom pose data, calculated by RealSense device *)
  RS2_STREAM_CONFIDENCE                       , (***< 4 bit per-pixel depth confidence level *)
  RS2_STREAM_COUNT
);
Trs2_stream_to_string = function(stream: Trs2_stream): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

type
(*** \brief A stream's format identifies how binary data is encoded within a frame. *)
Trs2_format = (
  RS2_FORMAT_ANY             , (***< When passed to enable stream, librealsense will try to provide best suited format *)
  RS2_FORMAT_Z16             , (***< 16-bit linear depth values. The depth is meters is equal to depth scale * pixel value. *)
  RS2_FORMAT_DISPARITY16     , (***< 16-bit float-point disparity values. Depth->Disparity conversion : Disparity = Baseline*FocalLength/Depth. *)
  RS2_FORMAT_XYZ32F          , (***< 32-bit floating point 3D coordinates. *)
  RS2_FORMAT_YUYV            , (***< 32-bit y0, u, y1, v data for every two pixels. Similar to YUV422 but packed in a different order - https://en.wikipedia.org/wiki/YUV *)
  RS2_FORMAT_RGB8            , (***< 8-bit red, green and blue channels *)
  RS2_FORMAT_BGR8            , (***< 8-bit blue, green, and red channels -- suitable for OpenCV *)
  RS2_FORMAT_RGBA8           , (***< 8-bit red, green and blue channels + constant alpha channel equal to FF *)
  RS2_FORMAT_BGRA8           , (***< 8-bit blue, green, and red channels + constant alpha channel equal to FF *)
  RS2_FORMAT_Y8              , (***< 8-bit per-pixel grayscale image *)
  RS2_FORMAT_Y16             , (***< 16-bit per-pixel grayscale image *)
  RS2_FORMAT_RAW10           , (***< Four 10 bits per pixel luminance values packed into a 5-byte macropixel *)
  RS2_FORMAT_RAW16           , (***< 16-bit raw image *)
  RS2_FORMAT_RAW8            , (***< 8-bit raw image *)
  RS2_FORMAT_UYVY            , (***< Similar to the standard YUYV pixel format, but packed in a different order *)
  RS2_FORMAT_MOTION_RAW      , (***< Raw data from the motion sensor *)
  RS2_FORMAT_MOTION_XYZ32F   , (***< Motion data packed as 3 32-bit float values, for X, Y, and Z axis *)
  RS2_FORMAT_GPIO_RAW        , (***< Raw data from the external sensors hooked to one of the GPIO's *)
  RS2_FORMAT_6DOF            , (***< Pose data packed as floats array, containing translation vector, rotation quaternion and prediction velocities and accelerations vectors *)
  RS2_FORMAT_DISPARITY32     , (***< 32-bit float-point disparity values. Depth->Disparity conversion : Disparity = Baseline*FocalLength/Depth *)
  RS2_FORMAT_Y10BPACK        , (***< 16-bit per-pixel grayscale image unpacked from 10 bits per pixel packed ([8:8:8:8:2222]) grey-scale image. The data is unpacked to LSB and padded with 6 zero bits *)
  RS2_FORMAT_DISTANCE        , (***< 32-bit float-point depth distance value.  *)
  RS2_FORMAT_MJPEG           , (***< Bitstream encoding for video in which an image of each frame is encoded as JPEG-DIB   *)
  RS2_FORMAT_Y8I             , (***< 8-bit per pixel interleaved. 8-bit left, 8-bit right.  *)
  RS2_FORMAT_Y12I            , (***< 12-bit per pixel interleaved. 12-bit left, 12-bit right. Each pixel is stored in a 24-bit word in little-endian order. *)
  RS2_FORMAT_INZI            , (***< multi-planar Depth 16bit + IR 10bit.  *)
  RS2_FORMAT_INVI            , (***< 8-bit IR stream.  *)
  RS2_FORMAT_W10             , (***< Grey-scale image as a bit-packed array. 4 pixel data stream taking 5 bytes *)
  RS2_FORMAT_Z16H            , (***< Variable-length Huffman-compressed 16-bit depth values. *)
  RS2_FORMAT_COUNT         (***< Number of enumeration values. Not a valid input: intended to be used in for-loops. *)
);

Trs2_format_to_string = function(format: Trs2_format): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

type
(*** \brief Cross-stream extrinsics: encodes the topology describing how the different devices are oriented. *)
Prs2_extrinsics = ^Trs2_extrinsics;
Trs2_extrinsics = record
  rotation: Array [0..8] of Single;(***< Column-major 3x3 rotation matrix *)
  translation: Array [0..2] of Single; (***< Three-element translation vector, in meters *)
end;

(***
 * Deletes sensors list, any sensors created from this list will remain unaffected
 * \param[in] info_list list to delete
 *)
Trs2_delete_sensor_list = procedure(info_list: Prs2_sensor_list); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Determines number of sensors in a list
 * \param[in] info_list The list of connected sensors captured using rs2_query_sensors
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return            Sensors count
 *)
Trs2_get_sensors_count = function(const info_list: Prs2_sensor_list;
                                  error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * delete relasense sensor
 * \param[in] sensor realsense sensor to delete
 *)
Trs2_delete_sensor = procedure(sensor: Prs2_sensor); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * create sensor by index
 * \param[in] index   the zero based index of sensor to retrieve
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return            the requested sensor, should be released by rs2_delete_sensor
 *)
Trs2_create_sensor = function(const list: Prs2_sensor_list;
                              index: Integer;
                              error: PPrs2_error): Prs2_sensor; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * This is a helper function allowing the user to discover the device from one of its sensors
 * \param[in] sensor     Pointer to a sensor
 * \param[out] error     if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return               new device wrapper for the device of the sensor. Needs to be released by delete_device
 *)
Trs2_create_device_from_sensor = function(const sensor: Prs2_sensor;
                                          error: PPrs2_error): Prs2_device; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * retrieve sensor specific information, like versions of various internal components
 * \param[in] sensor     the RealSense sensor
 * \param[in] info       camera info type to retrieve
 * \param[out] error     if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return               the requested camera info string, in a format specific to the device model
 *)
Trs2_get_sensor_info = function(const sensor: Prs2_sensor;
                                info: Trs2_camera_info;
                                error: PPrs2_error): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * check if specific sensor info is supported
 * \param[in] info    the parameter to check for support
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return                true if the parameter both exist and well-defined for the specific device
 *)
Trs2_supports_sensor_info = function(const sensor: Prs2_sensor;
                                     info: Trs2_camera_info;
                                     error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Test if the given sensor can be extended to the requested extension
 * \param[in] sensor  Realsense sensor
 * \param[in] extension The extension to which the sensor should be tested if it is extendable
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return non-zero value iff the sensor can be extended to the given extension
 *)
Trs2_is_sensor_extendable_to = function(const sensor: Prs2_sensor;
                                        extension: Trs2_extension;
                                        error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(*** When called on a depth sensor, this method will return the number of meters represented by a single depth unit
 * \param[in] sensor      depth sensor
 * \param[out] error      if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return                the number of meters represented by a single depth unit
 *)
Trs2_get_depth_scale = function(sensor: Prs2_sensor;
                                error: PPrs2_error): Single; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Retrieve the stereoscopic baseline value from frame. Applicable to stereo-based depth modules
 * \param[out] float  Stereoscopic baseline in millimeters
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_depth_stereo_frame_get_baseline = function(const frame_ref: Prs2_frame;
                                                error: PPrs2_error): Single; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Retrieve the stereoscopic baseline value from sensor. Applicable to stereo-based depth modules
 * \param[out] float  Stereoscopic baseline in millimeters
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_get_stereo_baseline = function(sensor: Prs2_sensor;
                                    error: PPrs2_error): Single; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * \brief sets the active region of interest to be used by auto-exposure algorithm
 * \param[in] sensor     the RealSense sensor
 * \param[in] min_x      lower horizontal bound in pixels
 * \param[in] min_y      lower vertical bound in pixels
 * \param[in] max_x      upper horizontal bound in pixels
 * \param[in] max_y      upper vertical bound in pixels
 * \param[out] error     if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_set_region_of_interest = procedure(const sensor: Prs2_sensor;
                                        min_x: Integer;
                                        min_y: Integer;
                                        max_x: Integer;
                                        max_y: Integer;
                                        error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * \brief gets the active region of interest to be used by auto-exposure algorithm
 * \param[in] sensor     the RealSense sensor
 * \param[out] min_x     lower horizontal bound in pixels
 * \param[out] min_y     lower vertical bound in pixels
 * \param[out] max_x     upper horizontal bound in pixels
 * \param[out] max_y     upper vertical bound in pixels
 * \param[out] error     if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_get_region_of_interest = procedure(const sensor: Prs2_sensor;
                                        min_x: PInteger;
                                        min_y: PInteger;
                                        max_x: PInteger;
                                        max_y: PInteger;
                                        error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * open subdevice for exclusive access, by committing to a configuration
 * \param[in] device relevant RealSense device
 * \param[in] profile    stream profile that defines single stream configuration
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_open = procedure(device: Prs2_sensor;
                      const profile: Prs2_stream_profile;
                      error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * open subdevice for exclusive access, by committing to composite configuration, specifying one or more stream profiles
 * this method should be used for interdependent  streams, such as depth and infrared, that have to be configured together
 * \param[in] device relevant RealSense device
 * \param[in] profiles  list of stream profiles discovered by get_stream_profiles
 * \param[in] count      number of simultaneous  stream profiles to configure
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_open_multiple = procedure(device: Prs2_sensor;
                               const profiles: PPrs2_stream_profile;
                               count: Integer;
                               error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * stop any streaming from specified subdevice
 * \param[in] sensor     RealSense device
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_close = procedure(const sensor: Prs2_sensor;
                       error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * start streaming from specified configured sensor
 * \param[in] sensor  RealSense device
 * \param[in] on_frame function pointer to register as per-frame callback
 * \param[in] user auxiliary  data the user wishes to receive together with every frame callback
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_start = procedure(const sensor: Prs2_sensor;
                       //on_frame: rs2_frame_callback_ptr;
                       user: Pointer;
                       error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * start streaming from specified configured sensor
 * \param[in] sensor  RealSense device
 * \param[in] callback callback object created from c++ application. ownership over the callback object is moved into the relevant streaming lock
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_start_cpp = procedure(const sensor: Prs2_sensor;
                           callback: Prs2_frame_callback;
                           error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * start streaming from specified configured sensor of specific stream to frame queue
 * \param[in] sensor  RealSense Sensor
 * \param[in] queue   frame-queue to store new frames into
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_start_queue = procedure(const sensor: Prs2_sensor;
                             queue: Prs2_frame_queue;
                             error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * stops streaming from specified configured device
 * \param[in] sensor  RealSense sensor
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_stop = procedure(const sensor: Prs2_sensor;
                      error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * set callback to get notifications from specified sensor
 * \param[in] sensor          RealSense device
 * \param[in] on_notification function pointer to register as per-notifications callback
 * \param[out] error          if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_set_notifications_callback = procedure(const sensor: Prs2_sensor;
                                            //on_notification: rs2_notification_callback_ptr;
                                            user: Pointer;
                                            error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * set callback to get notifications from specified device
 * \param[in] sensor  RealSense sensor
 * \param[in] callback callback object created from c++ application. ownership over the callback object is moved into the relevant subdevice lock
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_set_notifications_callback_cpp = procedure(const sensor: Prs2_sensor;
                                                callback: Prs2_notifications_callback;
                                                error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * retrieve description from notification handle
 * \param[in] notification      handle returned from a callback
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return            the notification description
 *)
Trs2_get_notification_description = function(notification: Prs2_notification;
                                             error: PPrs2_error): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * retrieve timestamp from notification handle
 * \param[in] notification      handle returned from a callback
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return            the notification timestamp
 *)
Trs2_get_notification_timestamp = function(notification: Prs2_notification;
                                           error: PPrs2_error): Trs2_time; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * retrieve severity from notification handle
 * \param[in] notification      handle returned from a callback
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return            the notification severity
 *)
Trs2_get_notification_severity = function(notification: Prs2_notification;
                                          error: PPrs2_error): Trs2_log_severity; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * retrieve category from notification handle
 * \param[in] notification      handle returned from a callback
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return            the notification category
 *)
Trs2_get_notification_category = function(notification: Prs2_notification;
                                          error: PPrs2_error): Trs2_notification_category; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * retrieve serialized data from notification handle
 * \param[in] notification      handle returned from a callback
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return            the serialized data (in JSON format)
 *)
Trs2_get_notification_serialized_data = function(notification: Prs2_notification;
                                                 error: PPrs2_error): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * check if physical subdevice is supported
 * \param[in] sensor  input RealSense subdevice
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return            list of stream profiles that given subdevice can provide, should be released by rs2_delete_profiles_list
 *)
Trs2_get_stream_profiles = function(sensor: Prs2_sensor;
                                    error: PPrs2_error): Prs2_stream_profile_list; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * check how subdevice is streaming
 * \param[in] sensor  input RealSense subdevice
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return            list of stream profiles that given subdevice is currently streaming, should be released by rs2_delete_profiles_list
 *)
Trs2_get_active_streams = function(sensor: Prs2_sensor;
                                   error: PPrs2_error): Prs2_stream_profile_list; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Get pointer to specific stream profile
 * \param[in] list        the list of supported profiles returned by rs2_get_supported_profiles
 * \param[in] index       the zero based index of the streaming mode
 * \param[out] error      if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_get_stream_profile = function(const list: Prs2_stream_profile_list;
                                   index: Integer;
                                   error: PPrs2_error): Prs2_stream_profile; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Extract common parameters of a stream profiles
 * \param[in] mode        input stream profile
 * \param[out] stream     stream type of the input profile
 * \param[out] format     binary data format of the input profile
 * \param[out] index      stream index the input profile in case there are multiple streams of the same type
 * \param[out] unique_id  identifier for the stream profile, unique within the application
 * \param[out] framerate  expected rate for data frames to arrive, meaning expected number of frames per second
 * \param[out] error      if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_get_stream_profile_data = procedure(const mode: Prs2_stream_profile;
                                         //stream: Prs2_stream;
                                         //format: Prs2_format;
                                         index: PInteger;
                                         unique_id: PInteger;
                                         framerate: PInteger;
                                         error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Override some of the parameters of the stream profile
 * \param[in] mode        input stream profile
 * \param[in] stream      stream type for the profile
 * \param[in] format      binary data format of the profile
 * \param[in] index       stream index the profile in case there are multiple streams of the same type
 * \param[out] error      if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_set_stream_profile_data = procedure(mode: Prs2_stream_profile;
                                         stream: Trs2_stream;
                                         index: Integer;
                                         format: Trs2_format;
                                         error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Creates a copy of stream profile, assigning new values to some of the fields
 * \param[in] mode        input stream profile
 * \param[in] stream      stream type for the profile
 * \param[in] format      binary data format of the profile
 * \param[in] index       stream index the profile in case there are multiple streams of the same type
 * \param[out] error      if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return                new stream profile, must be deleted by rs2_delete_stream_profile
 *)
Trs2_clone_stream_profile = function(const mode: Prs2_stream_profile;
                                     stream: Trs2_stream;
                                     index: Integer;
                                     format: Trs2_format;
                                     error: PPrs2_error): Prs2_stream_profile; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Creates a copy of stream profile, assigning new values to some of the fields
 * \param[in] mode        input stream profile
 * \param[in] stream      stream type for the profile
 * \param[in] format      binary data format of the profile
 * \param[in] width       new width for the profile
 * \param[in] height      new height for the profile
 * \param[in] intr        new intrinsics for the profile
 * \param[in] index       stream index the profile in case there are multiple streams of the same type
 * \param[out] error      if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return                new stream profile, must be deleted by rs2_delete_stream_profile
 *)
Trs2_clone_video_stream_profile = function(const mode: Prs2_stream_profile;
                                           stream: Trs2_stream;
                                           index: Integer;
                                           format: Trs2_format;
                                           width: Integer;
                                           height: Integer;
                                           const intr: Prs2_intrinsics;
                                           error: PPrs2_error): Prs2_stream_profile; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Delete stream profile allocated by rs2_clone_stream_profile
 * Should not be called on stream profiles returned by the device
 * \param[in] mode        input stream profile
 *)
Trs2_delete_stream_profile = procedure(mode: Prs2_stream_profile); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Try to extend stream profile to an extension type
 * \param[in] mode        input stream profile
 * \param[in] type        extension type, for example RS2_EXTENSION_VIDEO_STREAM_PROFILE
 * \param[out] error      if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return                non-zero if profile is extendable to specified extension, zero otherwise
 *)
Trs2_stream_profile_is = function(const mode: Prs2_stream_profile;
                                  type_: Trs2_extension;
                                  error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * When called on a video stream profile, will return the width and the height of the stream
 * \param[in] mode        input stream profile
 * \param[out] width      width in pixels of the video stream
 * \param[out] height     height in pixels of the video stream
 * \param[out] error      if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_get_video_stream_resolution = procedure(const mode: Prs2_stream_profile;
                                             width: PInteger;
                                             height: PInteger;
                                             error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Obtain the intrinsics of a specific stream configuration from the device.
 * \param[in] mode          input stream profile
 * \param[out] intrinsics   Pointer to the struct to store the data in
 * \param[out] error        If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_get_motion_intrinsics = procedure(const mode: Prs2_stream_profile;
                                       intrinsics: Prs2_motion_device_intrinsic;
                                       error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Returns non-zero if selected profile is recommended for the sensor
 * This is an optional hint we offer to suggest profiles with best performance-quality tradeof
 * \param[in] mode        input stream profile
 * \param[out] error      if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return                non-zero if selected profile is recommended for the sensor
 *)
Trs2_is_stream_profile_default = function(const mode: Prs2_stream_profile;
                                          error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * get the number of supported stream profiles
 * \param[in] list        the list of supported profiles returned by rs2_get_supported_profiles
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return number of supported subdevice profiles
 *)
Trs2_get_stream_profiles_count = function(const list: Prs2_stream_profile_list;
                                          error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * delete stream profiles list
 * \param[in] list        the list of supported profiles returned by rs2_get_supported_profiles
 *)
Trs2_delete_stream_profiles_list = procedure(list: Prs2_stream_profile_list); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * \param[in] from          origin stream profile
 * \param[in] to            target stream profile
 * \param[out] extrin       extrinsics from origin to target
 * \param[out] error        if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_get_extrinsics = procedure(const from: Prs2_stream_profile;
                                const to_: Prs2_stream_profile;
                                extrin: Prs2_extrinsics;
                                error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * \param[in] from          origin stream profile
 * \param[in] to            target stream profile
 * \param[out] extrin       extrinsics from origin to target
 * \param[out] error        if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_register_extrinsics = procedure(const from: Prs2_stream_profile;
                                     const to_: Prs2_stream_profile;
                                     extrin: Trs2_extrinsics;
                                     error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * \brief Override extrinsics of a given sensor that supports calibrated_sensor.
 *
 * This will affect extrinsics at the source device and may affect multiple profiles. Used for DEPTH_TO_RGB calibration.
 *
 * \param[in] sensor       The sensor
 * \param[in] extrinsics   Extrinsics from Depth to the named sensor
 * \param[out] error       If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_override_extrinsics = procedure(const sensor: Prs2_sensor;
                                     const extrinsics: Prs2_extrinsics;
                                     error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * When called on a video profile, returns the intrinsics of specific stream configuration
 * \param[in] mode          input stream profile
 * \param[out] intrinsics   resulting intrinsics for the video profile
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_get_video_stream_intrinsics = procedure(const mode: Prs2_stream_profile;
                                             intrinsics: Prs2_intrinsics;
                                             error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Returns the list of recommended processing blocks for a specific sensor.
 * Order and configuration of the blocks are decided by the sensor
 * \param[in] sensor          input sensor
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return list of supported sensor recommended processing blocks
 *)
Trs2_get_recommended_processing_blocks = function(sensor: Prs2_sensor;
                                                  error: PPrs2_error): Prs2_processing_block_list; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Returns specific processing blocks from processing blocks list
 * \param[in] list           the processing blocks list
 * \param[in] index          the requested processing block
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return processing block
 *)
Trs2_get_processing_block = function(const list: Prs2_processing_block_list;
                                     index: Integer;
                                     error: PPrs2_error): Prs2_processing_block; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Returns the processing blocks list size
 * \param[in] list           the processing blocks list
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return the processing block list size
 *)
Trs2_get_recommended_processing_blocks_count = function(const list: Prs2_processing_block_list;
                                                        error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Deletes processing blocks list
 * \param[in] list list to delete
 *)
Trs2_delete_recommended_processing_blocks = procedure(list: Prs2_processing_block_list); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Imports a localization map from file to tm2 tracking device
 * \param[in]  sensor        TM2 position-tracking sensor
 * \param[in]  lmap_blob     Localization map raw buffer, serialized
 * \param[in]  blob_size     The buffer's size in bytes
 * \param[out] error         If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return                   Non-zero if succeeded, otherwise 0
 *)
Trs2_import_localization_map = function(const sensor: Prs2_sensor;
                                        const lmap_blob: Byte;
                                        blob_size: FixedUInt;
                                        error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Extract and store the localization map of tm2 tracking device to file
 * \param[in]  sensor        TM2 position-tracking sensor
 * \param[in]  lmap_fname    The file name of the localization map
 * \param[out] error         If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return                   Device's response in a rs2_raw_data_buffer, which should be released by rs2_delete_raw_data
 *)
 //void rs2_export_localization_map(const rs2_sensor* sensor, const char* lmap_fname, rs2_error** error);
Trs2_export_localization_map = function(const sensor: Prs2_sensor;
                                        error: PPrs2_error): Prs2_raw_data_buffer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Create a named location tag
 * \param[in]  sensor    T2xx position-tracking sensor
 * \param[in]  guid      Null-terminated string of up to 127 characters
 * \param[in]  pos       Position in meters, relative to the current tracking session
 * \param[in]  orient    Quaternion orientation, expressed the the coordinate system of the current tracking session
 * \param[out] error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return               Non-zero if succeeded, otherwise 0
 *)
Trs2_set_static_node = function(const sensor: Prs2_sensor;
                                const guid: PByte;
                                const pos: Trs2_vector;
                                const orient: Trs2_quaternion;
                                error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Retrieve a named location tag
 * \param[in]  sensor    T2xx position-tracking sensor
 * \param[in]  guid      Null-terminated string of up to 127 characters
 * \param[out] pos       Position in meters of the tagged (stored) location
 * \param[out] orient    Quaternion orientation of the tagged (stored) location
 * \param[out] error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return               Non-zero if succeeded, otherwise 0
 *)
Trs2_get_static_node = function(const sensor: Prs2_sensor;
                                const guid: PByte;
                                pos: Prs2_vector;
                                orient: Prs2_quaternion;
                                error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Remove a named location tag
 * \param[in]  sensor    T2xx position-tracking sensor
 * \param[in]  guid      Null-terminated string of up to 127 characters
 * \param[out] error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return               Non-zero if succeeded, otherwise 0
 *)
Trs2_remove_static_node = function(const sensor: Prs2_sensor;
                                   const guid: PByte;
                                   error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(*** Load Wheel odometer settings from host to device
 * \param[in] odometry_config_buf   odometer configuration/calibration blob serialized from jsom file
 * \return true on success
 *)
Trs2_load_wheel_odometry_config = function(const sensor: Prs2_sensor;
                                           const odometry_config_buf: PByte;
                                           blob_size: FixedUInt;
                                           error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(*** Send wheel odometry data for each individual sensor (wheel)
 * \param[in] wo_sensor_id       - Zero-based index of (wheel) sensor with the same type within device
 * \param[in] frame_num          - Monotonocally increasing frame number, managed per sensor.
 * \param[in] translational_velocity   - Translational velocity of the wheel sensor [meter/sec]
 * \return true on success
 *)
Trs2_send_wheel_odometry = function(const sensor: Prs2_sensor;
                                    wo_sensor_id: Byte;
                                    frame_num: FixedUInt;
                                    const translational_velocity: Trs2_vector;
                                    error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Set intrinsics of a given sensor
 * \param[in] sensor       The RealSense device
 * \param[in] profile      Target stream profile
 * \param[in] intrinsics   Intrinsics value to be written to the device
 * \param[out] error       If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_set_intrinsics = procedure(const sensor: Prs2_sensor;
                                const profile: Prs2_stream_profile;
                                const intrinsics: Prs2_intrinsics;
                                error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * \brief Override intrinsics of a given sensor that supports calibrated_sensor.
 *
 * This will affect intrinsics at the source and may affect multiple profiles. Used for DEPTH_TO_RGB calibration.
 *
 * \param[in] sensor       The RealSense device
 * \param[in] intrinsics   Intrinsics value to be written to the sensor
 * \param[out] error       If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_override_intrinsics = procedure(const sensor: Prs2_sensor;
                                     const intrinsics: Prs2_intrinsics;
                                     error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Set extrinsics between two sensors
 * \param[in]  from_sensor  Origin sensor
 * \param[in]  from_profile Origin profile
 * \param[in]  to_sensor    Target sensor
 * \param[in]  to_profile   Target profile
 * \param[out] extrinsics   Extrinsics from origin to target
 * \param[out] error        If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_set_extrinsics = procedure(const sensor: Prs2_sensor;
                                const from_profile: Prs2_stream_profile;
                                to_sensor: Prs2_sensor;
                                const to_profile: Prs2_stream_profile;
                                const extrinsics: Prs2_extrinsics;
                                error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Get the DSM parameters for a sensor
 * \param[in]  sensor        Sensor that supports the CALIBRATED_SENSOR extension
 * \param[out] p_params_out  Pointer to the structure that will get the DSM parameters
 * \param[out] error         If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_get_dsm_params = procedure(sensor: Prs2_sensor;
                                p_params_out: Prs2_dsm_params;
                                error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Set the sensor DSM parameters
 * This should ideally be done when the stream is NOT running. If it is, the
 * parameters may not take effect immediately.
 * \param[in]  sensor        Sensor that supports the CALIBRATED_SENSOR extension
 * \param[out] p_params      Pointer to the structure that contains the DSM parameters
 * \param[out] error         If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_override_dsm_params = procedure(sensor: Prs2_sensor;
                                     p_params: Prs2_dsm_params;
                                     error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Reset the sensor DSM parameters
 * This should ideally be done when the stream is NOT running. May not take effect immediately.
 * \param[in]  sensor        Sensor that supports the CALIBRATED_SENSOR extension
 * \param[out] error         If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_reset_sensor_calibration = procedure(sensor: Prs2_sensor;
                                          error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Set motion device intrinsics
 * \param[in]  sensor       Motion sensor
 * \param[in]  profile      Motion stream profile
 * \param[out] intrinsics   Pointer to the struct to store the data in
 * \param[out] error        If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_set_motion_device_intrinsics = procedure(const sensor: Prs2_sensor;
                                              const profile: Prs2_stream_profile;
                                              const intrinsics: Prs2_motion_device_intrinsic;
                                              error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};


//------------------------------------------------------------------------------

(***
 * Determines number of devices in a list.
 * \param[in]  info_list The list of connected devices captured using rs2_query_devices
 * \param[out] error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return               Device count
 *)
Trs2_get_device_count = function(const info_list: Prs2_device_list;
                                 error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Deletes device list, any devices created using this list will remain unaffected.
 * \param[in]  info_list List to delete
 *)
Trs2_delete_device_list = procedure(info_list: Prs2_device_list); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Checks if a specific device is contained inside a device list.
 * \param[in]  info_list The list of devices to check in
 * \param[in]  device    RealSense device to check for
 * \param[out] error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return               True if the device is in the list and false otherwise
 *)
Trs2_device_list_contains = function(const info_list: Prs2_device_list;
                                     const device: Prs2_device;
                                     error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Creates a device by index. The device object represents a physical camera and provides the means to manipulate it.
 * \param[in]  info_list the list containing the device to retrieve
 * \param[in]  index     The zero based index of device to retrieve
 * \param[out] error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return               The requested device, should be released by rs2_delete_device
 *)
Trs2_create_device = function(const info_list: Prs2_device_list;
                              index: Integer;
                              error: PPrs2_error): Prs2_device; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Delete RealSense device
 * \param[in]  device    Realsense device to delete
 *)
Trs2_delete_device = procedure(device: Prs2_device); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Retrieve camera specific information, like versions of various internal components.
 * \param[in]  device    The RealSense device
 * \param[in]  info      Camera info type to retrieve
 * \param[out] error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return               The requested camera info string, in a format specific to the device model
 *)
Trs2_get_device_info = function(const device: Prs2_device;
                                info: Trs2_camera_info;
                                error: PPrs2_error): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Check if a camera supports a specific camera info type.
 * \param[in]  device    The RealSense device to check
 * \param[in]  info      The parameter to check for support
 * \param[out] error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return               True if the parameter both exist and well-defined for the specific device
 *)
Trs2_supports_device_info = function(const device: Prs2_device;
                                     info: Trs2_camera_info;
                                     error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Send hardware reset request to the device. The actual reset is asynchronous.
 * Note: Invalidates all handles to this device.
 * \param[in]  device   The RealSense device to reset
 * \param[out] error    If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_hardware_reset = procedure(const device: Prs2_device;
                                error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Send raw data to device
 * \param[in]  device                    RealSense device to send data to
 * \param[in]  raw_data_to_send          Raw data to be sent to device
 * \param[in]  size_of_raw_data_to_send  Size of raw_data_to_send in bytes
 * \param[out] error                     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return                               Device's response in a rs2_raw_data_buffer, which should be released by rs2_delete_raw_data
 *)
Trs2_send_and_receive_raw_data = function(device: Prs2_device;
                                          raw_data_to_send: Pointer;
                                          size_of_raw_data_to_send: Cardinal;
                                          error: PPrs2_error): Prs2_raw_data_buffer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Test if the given device can be extended to the requested extension.
 * \param[in]  device    Realsense device
 * \param[in]  extension The extension to which the device should be tested if it is extendable
 * \param[out] error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return               Non-zero value iff the device can be extended to the given extension
 *)
Trs2_is_device_extendable_to = function(const device: Prs2_device;
                                        extension: Trs2_extension;
                                        error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Create a static snapshot of all connected sensors within a specific device.
 * \param[in]  device    Specific RealSense device
 * \param[out] error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return               The list of sensors, should be released by rs2_delete_sensor_list
 *)
Trs2_query_sensors = function(const device: Prs2_device;
                              error: PPrs2_error): Prs2_sensor_list; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Enter the given device into loopback operation mode that uses the given file as input for raw data
 * \param[in]  device     Device to enter into loopback operation mode
 * \param[in]  from_file  Path to bag file with raw data for loopback
 * \param[out] error      If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_loopback_enable = procedure(const device: Prs2_device;
                                 const from_file: PAnsiChar;
                                 error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Restores the given device into normal operation mode
 * \param[in]  device     Device to restore to normal operation mode
 * \param[out] error      If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_loopback_disable = procedure(const device: Prs2_device;
                               error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Checks if the device is in loopback mode or not
 * \param[in]  device     Device to check for operation mode
 * \param[out] error      If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return true if the device is in loopback operation mode
 *)
Trs2_loopback_is_enabled = function(const device: Prs2_device;
                                    error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Connects to a given tm2 controller
 * \param[in]  device     Device to connect to the controller
 * \param[in]  mac_addr   The MAC address of the desired controller
 * \param[out] error      If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_connect_tm2_controller = procedure(const device: Prs2_device;
                                        const mac_addr: PByte;
                                        error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Disconnects a given tm2 controller
 * \param[in]  device     Device to disconnect the controller from
 * \param[in]  id         The ID of the desired controller
 * \param[out] error      If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_disconnect_tm2_controller = procedure(const device: Prs2_device;
                                           id: Integer;
                                           error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};


(***
 * Reset device to factory calibration
 * \param[in] device       The RealSense device
 * \param[out] error       If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_reset_to_factory_calibration = procedure(const device: Prs2_device;
                                              e: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Write calibration to device's EEPROM
 * \param[in] device       The RealSense device
 * \param[out] error       If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_write_calibration = procedure(const device: Prs2_device;
                                   e: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Update device to the provided firmware, the device must be extendable to RS2_EXTENSION_UPDATABLE.
 * This call is executed on the caller's thread and it supports progress notifications via the optional callback.
 * \param[in]  device        Device to update
 * \param[in]  fw_image      Firmware image buffer
 * \param[in]  fw_image_size Firmware image buffer size
 * \param[in]  callback      Optional callback for update progress notifications, the progress value is normailzed to 1
 * \param[out] error         If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_update_firmware_cpp = procedure(const device: Prs2_device;
                                     const fw_image: Pointer;
                                     fw_image_size: Integer;
                                     callback: Prs2_update_progress_callback;
                                     error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Update device to the provided firmware, the device must be extendable to RS2_EXTENSION_UPDATABLE.
 * This call is executed on the caller's thread and it supports progress notifications via the optional callback.
 * \param[in]  device        Device to update
 * \param[in]  fw_image      Firmware image buffer
 * \param[in]  fw_image_size Firmware image buffer size
 * \param[in]  callback      Optional callback for update progress notifications, the progress value is normailzed to 1
 * \param[in]  client_data   Optional client data for the callback
 * \param[out] error         If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_update_firmware = procedure(const device: Prs2_device;
                                 const fw_image: Pointer;
                                 fw_image_size: Integer;
                                 //callback: rs2_update_progress_callback_ptr;
                                 client_data: Pointer;
                                 error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Create backup of camera flash memory. Such backup does not constitute valid firmware image, and cannot be
 * loaded back to the device, but it does contain all calibration and device information.
 * \param[in]  device        Device to update
 * \param[in]  callback      Optional callback for update progress notifications, the progress value is normailzed to 1
 * \param[out] error         If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_create_flash_backup_cpp = function(const device: Prs2_device;
                                        callback: Prs2_update_progress_callback;
                                        error: PPrs2_error): Prs2_raw_data_buffer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Create backup of camera flash memory. Such backup does not constitute valid firmware image, and cannot be
 * loaded back to the device, but it does contain all calibration and device information.
 * \param[in]  device        Device to update
 * \param[in]  callback      Optional callback for update progress notifications, the progress value is normailzed to 1
 * \param[in]  client_data   Optional client data for the callback
 * \param[out] error         If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_create_flash_backup = function(const device: Prs2_device;
                                    //callback: rs2_update_progress_callback_ptr;
                                    client_data: Pointer;
                                    error: PPrs2_error): Prs2_raw_data_buffer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

const
  RS2_UNSIGNED_UPDATE_MODE_UPDATE     = 0;
  RS2_UNSIGNED_UPDATE_MODE_READ_ONLY  = 1;
  RS2_UNSIGNED_UPDATE_MODE_FULL       = 2;

type
(***
 * Update device to the provided firmware by writing raw data directly to the flash, this command can be executed only on unlocked camera.
 * The device must be extendable to RS2_EXTENSION_UPDATABLE.
 * This call is executed on the caller's thread and it supports progress notifications via the optional callback.
 * \param[in]  device        Device to update
 * \param[in]  fw_image      Firmware image buffer
 * \param[in]  fw_image_size Firmware image buffer size
 * \param[in]  callback      Optional callback for update progress notifications, the progress value is normailzed to 1
 * \param[in]  update_mode   Select one of RS2_UNSIGNED_UPDATE_MODE, WARNING!!! setting to any option other than RS2_UNSIGNED_UPDATE_MODE_UPDATE will make this call unsafe and might damage the camera
 * \param[out] error         If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_update_firmware_unsigned_cpp = procedure(const device: Prs2_device;
                                              const fw_image: Pointer;
                                              fw_image_size: Integer;
                                              callback: Prs2_update_progress_callback;
                                              update_mode: Integer;
                                              error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Update device to the provided firmware by writing raw data directly to the flash, this command can be executed only on unlocked camera.
 * The device must be extendable to RS2_EXTENSION_UPDATABLE.
 * This call is executed on the caller's thread and it supports progress notifications via the optional callback.
 * \param[in]  device        Device to update
 * \param[in]  fw_image      Firmware image buffer
 * \param[in]  fw_image_size Firmware image buffer size
 * \param[in]  callback      Optional callback for update progress notifications, the progress value is normailzed to 1
 * \param[in]  client_data   Optional client data for the callback
 * \param[in]  update_mode   Select one of RS2_UNSIGNED_UPDATE_MODE, WARNING!!! setting to any option other than RS2_UNSIGNED_UPDATE_MODE_UPDATE will make this call unsafe and might damage the camera
 * \param[out] error         If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_update_firmware_unsigned = procedure(const device: Prs2_device;
                                          const fw_image: Pointer;
                                          fw_image_size: Integer;
                                          //callback: rs2_update_progress_callback_ptr;
                                          client_data: Pointer;
                                          update_mode: Integer;
                                          error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Enter the device to update state, this will cause the updatable device to disconnect and reconnect as update device.
 * \param[in]  device     Device to update
 * \param[out] error      If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_enter_update_state = procedure(const device: Prs2_device;
                                    error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * This will improve the depth noise.
 * \param[in] json_content       Json string to configure speed on chip calibration parameters:
                                     {
                                       "speed": 3,
                                       "scan parameter": 0,
                                       "data sampling": 0
                                     }
                                     speed - value can be one of: Very fast = 0, Fast = 1, Medium = 2, Slow = 3, White wall = 4, default is  Slow
                                     scan_parameter - value can be one of: Py scan (default) = 0, Rx scan = 1
                                     data_sampling - value can be one of:polling data sampling = 0, interrupt data sampling = 1
                                     if json is nullptr it will be ignored and calibration will use the default parameters
 * \param[out] health            Calibration Health-Check captures how far camera calibration is from the optimal one
                                 [0, 0.25) - Good
                                 [0.25, 0.75) - Can be Improved
                                 [0.75, ) - Requires Calibration
 * \param[in] callback           Optional callback to get progress notifications
 * \param[in] timeout_ms         Timeout in ms (use 5000 msec unless instructed otherwise)
 * \return                       New calibration table
 *)
Trs2_run_on_chip_calibration_cpp = function(device: Prs2_device;
                                            const json_content: Pointer;
                                            content_size: Integer;
                                            health: PSingle;
                                            progress_callback: Prs2_update_progress_callback;
                                            timeout_ms: Integer;
                                            error: PPrs2_error): Prs2_raw_data_buffer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * This will improve the depth noise.
 * \param[in] json_content       Json string to configure speed on chip calibration parameters:
                                     {
                                       "speed": 3,
                                       "scan parameter": 0,
                                       "data sampling": 0
                                     }
                                     speed - value can be one of: Very fast = 0, Fast = 1, Medium = 2, Slow = 3, White wall = 4, default is  Slow
                                     scan_parameter - value can be one of: Py scan (default) = 0, Rx scan = 1
                                     data_sampling - value can be one of:polling data sampling = 0, interrupt data sampling = 1
                                     if json is nullptr it will be ignored and calibration will use the default parameters
 * \param[out] health            Calibration Health-Check captures how far camera calibration is from the optimal one
                                 [0, 0.25) - Good
                                 [0.25, 0.75) - Can be Improved
                                 [0.75, ) - Requires Calibration
 * \param[in]  callback          Optional callback for update progress notifications, the progress value is normailzed to 1
 * \param[in]  client_data       Optional client data for the callback
 * \param[in] timeout_ms         Timeout in ms (use 5000 msec unless instructed otherwise)
 * \return                       New calibration table
 *)
Trs2_run_on_chip_calibration = function(device: Prs2_device;
                                        const json_content: Pointer;
                                        content_size: Integer;
                                        health: PSingle;
                                        //callback: rs2_update_progress_callback_ptr;
                                        client_data: Pointer;
                                        timeout_ms: Integer;
                                        error: PPrs2_error): Prs2_raw_data_buffer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * This will adjust camera absolute distance to flat target. User needs to enter the known ground truth.
 * \param[in] ground_truth_mm     Ground truth in mm must be between 2500 - 2000000
 * \param[in] json_content        Json string to configure tare calibration parameters:
                                     {
                                       "average step count": 20,
                                       "step count": 20,
                                       "accuracy": 2,
                                       "scan parameter": 0,
                                       "data sampling": 0
                                     }
                                     average step count - number of frames to average, must be between 1 - 30, default = 20
                                     step count - max iteration steps, must be between 5 - 30, default = 10
                                     accuracy - Subpixel accuracy level, value can be one of: Very high = 0 (0.025%), High = 1 (0.05%), Medium = 2 (0.1%), Low = 3 (0.2%), Default = Very high (0.025%), default is very high (0.025%)
                                     scan_parameter - value can be one of: Py scan (default) = 0, Rx scan = 1
                                     data_sampling - value can be one of:polling data sampling = 0, interrupt data sampling = 1
                                     if json is nullptr it will be ignored and calibration will use the default parameters
 * \param[in]  content_size        Json string size if its 0 the json will be ignored and calibration will use the default parameters
 * \param[in]  callback            Optional callback to get progress notifications
 * \param[in] timeout_ms          Timeout in ms (use 5000 msec unless instructed otherwise)
 * \return                         New calibration table
 *)
Trs2_run_tare_calibration_cpp = function(dev: Prs2_device;
                                         ground_truth_mm: Single;
                                         const json_content: Pointer;
                                         content_size: Integer;
                                         progress_callback: Prs2_update_progress_callback;
                                         timeout_ms: Integer;
                                         error: PPrs2_error): Prs2_raw_data_buffer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

type
(***
 * Used in device_calibration; enumerates the different calibration types
 * available for that extension.
 *)
Trs2_calibration_type = (
    RS2_CALIBRATION_AUTO_DEPTH_TO_RGB,
    RS2_CALIBRATION_MANUAL_DEPTH_TO_RGB,
    RS2_CALIBRATION_TYPE_COUNT
);
Trs2_calibration_type_to_string = function(calibration_type: Trs2_calibration_type): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

type
(***
 * Used in device_calibration with rs2_calibration_change_callback
 *)
Trs2_calibration_status = (
    // Anything >= 0 is not an issue
    RS2_CALIBRATION_TRIGGERED      =  0,  // AC triggered and is active; conditions are valid
    RS2_CALIBRATION_SPECIAL_FRAME  =  1,  // Special frame received; expect a frame-drop!
    RS2_CALIBRATION_STARTED        =  2,  // Have all frames in hand; starting processing
    RS2_CALIBRATION_NOT_NEEDED     =  3,  // Finished; existing calibration within tolerances; nothing done!
    RS2_CALIBRATION_SUCCESSFUL     =  4  // Finished; have new calibration in-hand

    // TODO:
    //RS2_CALIBRATION_RETRY          = -1,  // Initiating retry (asked for a new special frame)
    //RS2_CALIBRATION_FAILED         = -2,  // Unexpected: exception, device removed, stream stopped, etc.
    //RS2_CALIBRATION_SCENE_INVALID  = -3,  // Scene was not good enough for calibration; will retry
    //RS2_CALIBRATION_BAD_RESULT     = -4,  // Calibration finished, but results aren't good; will retry
    //RS2_CALIBRATION_BAD_CONDITIONS = -5,  // Trigger was attempted but conditions (temp/APD) were invalid (still inactive)

    //RS2_CALIBRATION_STATUS_FIRST   = -5,
    //RS2_CALIBRATION_STATUS_LAST    =  4,
    //RS2_CALIBRATION_STATUS_COUNT = RS2_CALIBRATION_STATUS_LAST - RS2_CALIBRATION_STATUS_FIRST + 1,
);
Trs2_calibration_status_to_string = function(calibration_status: Trs2_calibration_status): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

//typedef struct rs2_calibration_change_callback rs2_calibration_change_callback;
//typedef void (*rs2_calibration_change_callback_ptr)(rs2_calibration_status, void* arg);

(***
 * Adds a callback for a sensor that gets called when calibration (intrinsics) changes, e.g. due to auto-calibration
 * \param[in] sensor        the sensor
 * \param[in] callback      the C callback function that gets called
 * \param[in] user          user argument that gets passed to the callback function
 * \param[out] error        if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_register_calibration_change_callback = procedure(dev: Prs2_device;
                                                      //callback: rs2_calibration_change_callback_ptr;
                                                      user: Pointer;
                                                      error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Adds a callback for a sensor that gets called when calibration (intrinsics) changes, e.g. due to auto-calibration
 * \param[in] sensor        the sensor
 * \param[in] callback      the C++ callback interface that gets called
 * \param[out] error        if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_register_calibration_change_callback_cpp = procedure(dev: Prs2_device;
                                                          //callback: Prs2_calibration_change_callback;
                                                          error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Triggers calibration of the given type
 * \param[in] dev           the device
 * \param[in] type          the type of calibration requested
 * \param[out] error        if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_trigger_device_calibration = procedure(dev: Prs2_device;
                                            type_: Trs2_calibration_type;
                                            error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * This will adjust camera absolute distance to flat target. User needs to enter the known ground truth.
 * \param[in] ground_truth_mm     Ground truth in mm must be between 2500 - 2000000
 * \param[in] json_content        Json string to configure tare calibration parameters:
                                    {
                                      "average_step_count": 20,
                                      "step count": 20,
                                      "accuracy": 2,
                                      "scan parameter": 0,
                                      "data sampling": 0
                                    }
                                    average step count - number of frames to average, must be between 1 - 30, default = 20
                                    step count - max iteration steps, must be between 5 - 30, default = 10
                                    accuracy - Subpixel accuracy level, value can be one of: Very high = 0 (0.025%), High = 1 (0.05%), Medium = 2 (0.1%), Low = 3 (0.2%), Default = Very high (0.025%), default is very high (0.025%)
                                    scan_parameter - value can be one of: Py scan (default) = 0, Rx scan = 1
                                    data_sampling - value can be one of:polling data sampling = 0, interrupt data sampling = 1
                                    if json is nullptr it will be ignored and calibration will use the default parameters
 * \param[in]  content_size       Json string size if its 0 the json will be ignored and calibration will use the default parameters
 * \param[in]  callback           Optional callback for update progress notifications, the progress value is normailzed to 1
 * \param[in]  client_data        Optional client data for the callback
 * \param[in] timeout_ms          Timeout in ms (use 5000 msec unless instructed otherwise)
 * \return                        New calibration table
 *)
Trs2_run_tare_calibration = function(dev: Prs2_device;
                                     ground_truth_mm: Single;
                                     const json_content: Pointer;
                                     content_size: Integer;
                                     //callback: rs2_update_progress_callback_ptr;
                                     client_data: Pointer;
                                     timeout_ms: Integer;
                                     error: PPrs2_error): Prs2_raw_data_buffer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 *  Read current calibration table from flash.
 * \return    Calibration table
 *)
Trs2_get_calibration_table = function(const dev: Prs2_device;
                                      error: PPrs2_error): Prs2_raw_data_buffer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 *  Set current table to dynamic area.
 * \param[in]     Calibration table
 *)
Trs2_set_calibration_table = procedure(const dev: Prs2_device;
                                       const calibration: Pointer;
                                       calibration_size: Integer;
                                       error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(** Serialize JSON content, returns ASCII-serialized JSON string on success. otherwise nullptr *)
Trs2_serialize_json = function(dev: Prs2_device;
                               error: PPrs2_error): Prs2_raw_data_buffer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(** Load JSON and apply advanced-mode controls *)
Trs2_load_json = procedure(dev: Prs2_device;
                           const json_content: Pointer;
                           content_size: Cardinal;
                           error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

//------------------------------------------------------------------------------
type
(*** \brief Specifies the clock in relation to which the frame timestamp was measured. *)
Trs2_timestamp_domain = (
    RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK, (***< Frame timestamp was measured in relation to the camera clock *)
    RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME,(***< Frame timestamp was measured in relation to the OS system clock *)
    RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME,(***< Frame timestamp was measured in relation to the camera clock and converted to OS system clock by constantly measure the difference*)
    RS2_TIMESTAMP_DOMAIN_COUNT       (***< Number of enumeration values. Not a valid input: intended to be used in for-loops. *)
);
Trs2_timestamp_domain_to_string = function(info: Trs2_timestamp_domain): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

type
(*** \brief Per-Frame-Metadata is the set of read-only properties that might be exposed for each individual frame. *)
Trs2_frame_metadata_value = (
    RS2_FRAME_METADATA_FRAME_COUNTER                        , (***< A sequential index managed per-stream. Integer value*)
    RS2_FRAME_METADATA_FRAME_TIMESTAMP                      , (***< Timestamp set by device clock when data readout and transmit commence. usec*)
    RS2_FRAME_METADATA_SENSOR_TIMESTAMP                     , (***< Timestamp of the middle of sensor's exposure calculated by device. usec*)
    RS2_FRAME_METADATA_ACTUAL_EXPOSURE                      , (***< Sensor's exposure width. When Auto Exposure (AE) is on the value is controlled by firmware. usec*)
    RS2_FRAME_METADATA_GAIN_LEVEL                           , (***< A relative value increasing which will increase the Sensor's gain factor. \
                                                              When AE is set On, the value is controlled by firmware. Integer value*)
    RS2_FRAME_METADATA_AUTO_EXPOSURE                        , (***< Auto Exposure Mode indicator. Zero corresponds to AE switched off. *)
    RS2_FRAME_METADATA_WHITE_BALANCE                        , (***< White Balance setting as a color temperature. Kelvin degrees*)
    RS2_FRAME_METADATA_TIME_OF_ARRIVAL                      , (***< Time of arrival in system clock *)
    RS2_FRAME_METADATA_TEMPERATURE                          , (***< Temperature of the device, measured at the time of the frame capture. Celsius degrees *)
    RS2_FRAME_METADATA_BACKEND_TIMESTAMP                    , (***< Timestamp get from uvc driver. usec*)
    RS2_FRAME_METADATA_ACTUAL_FPS                           , (***< Actual fps *)
    RS2_FRAME_METADATA_FRAME_LASER_POWER                    , (***< Laser power value 0-360. *)
    RS2_FRAME_METADATA_FRAME_LASER_POWER_MODE               , (***< Laser power mode. Zero corresponds to Laser power switched off and one for switched on. deprecated, replaced by RS2_FRAME_METADATA_FRAME_EMITTER_MODE*)
    RS2_FRAME_METADATA_EXPOSURE_PRIORITY                    , (***< Exposure priority. *)
    RS2_FRAME_METADATA_EXPOSURE_ROI_LEFT                    , (***< Left region of interest for the auto exposure Algorithm. *)
    RS2_FRAME_METADATA_EXPOSURE_ROI_RIGHT                   , (***< Right region of interest for the auto exposure Algorithm. *)
    RS2_FRAME_METADATA_EXPOSURE_ROI_TOP                     , (***< Top region of interest for the auto exposure Algorithm. *)
    RS2_FRAME_METADATA_EXPOSURE_ROI_BOTTOM                  , (***< Bottom region of interest for the auto exposure Algorithm. *)
    RS2_FRAME_METADATA_BRIGHTNESS                           , (***< Color image brightness. *)
    RS2_FRAME_METADATA_CONTRAST                             , (***< Color image contrast. *)
    RS2_FRAME_METADATA_SATURATION                           , (***< Color image saturation. *)
    RS2_FRAME_METADATA_SHARPNESS                            , (***< Color image sharpness. *)
    RS2_FRAME_METADATA_AUTO_WHITE_BALANCE_TEMPERATURE       , (***< Auto white balance temperature Mode indicator. Zero corresponds to automatic mode switched off. *)
    RS2_FRAME_METADATA_BACKLIGHT_COMPENSATION               , (***< Color backlight compensation. Zero corresponds to switched off. *)
    RS2_FRAME_METADATA_HUE                                  , (***< Color image hue. *)
    RS2_FRAME_METADATA_GAMMA                                , (***< Color image gamma. *)
    RS2_FRAME_METADATA_MANUAL_WHITE_BALANCE                 , (***< Color image white balance. *)
    RS2_FRAME_METADATA_POWER_LINE_FREQUENCY                 , (***< Power Line Frequency for anti-flickering Off/50Hz/60Hz/Auto. *)
    RS2_FRAME_METADATA_LOW_LIGHT_COMPENSATION               , (***< Color lowlight compensation. Zero corresponds to switched off. *)
    RS2_FRAME_METADATA_FRAME_EMITTER_MODE                   , (***< Emitter mode: 0 - all emitters disabled. 1 - laser enabled. 2 - auto laser enabled (opt). 3 - LED enabled (opt).*)
    RS2_FRAME_METADATA_FRAME_LED_POWER                      , (***< Led power value 0-360. *)
    RS2_FRAME_METADATA_RAW_FRAME_SIZE                       , (***< The number of transmitted payload bytes, not including metadata *)
    RS2_FRAME_METADATA_GPIO_INPUT_DATA                      , (***< GPIO input data *)
    RS2_FRAME_METADATA_SEQUENCE_NAME                         , (***< sub-preset id *)
    RS2_FRAME_METADATA_SEQUENCE_ID                , (***< sub-preset sequence id *)
    RS2_FRAME_METADATA_SEQUENCE_SIZE              , (***< sub-preset sequence size *)
    RS2_FRAME_METADATA_COUNT
);
Trs2_frame_metadata_to_string = function(metadata: Trs2_frame_metadata_value): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};
Trs2_frame_metadata_value_to_string = function(metadata: Trs2_frame_metadata_value): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * retrieve metadata from frame handle
 * \param[in] frame      handle returned from a callback
 * \param[in] frame_metadata  the rs2_frame_metadata whose latest frame we are interested in
 * \param[out] error         if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return            the metadata value
 *)
Trs2_get_frame_metadata = function(const frame: Prs2_frame;
                                   frame_metadata: Trs2_frame_metadata_value;
                                   error: PPrs2_error): Trs2_metadata_type; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * determine device metadata
 * \param[in] frame             handle returned from a callback
 * \param[in] frame_metadata    the metadata to check for support
 * \param[out] error         if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return                true if device has this metadata
 *)
Trs2_supports_frame_metadata = function(const frame: Prs2_frame;
                                        frame_metadata: Trs2_frame_metadata_value;
                                        error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * retrieve timestamp domain from frame handle. timestamps can only be comparable if they are in common domain
 * (for example, depth timestamp might come from system time while color timestamp might come from the device)
 * this method is used to check if two timestamp values are comparable (generated from the same clock)
 * \param[in] frameset   handle returned from a callback
 * \param[out] error     if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return               the timestamp domain of the frame (camera / microcontroller / system time)
 *)
Trs2_get_frame_timestamp_domain = function(const frameset: Prs2_frame;
                                           error: PPrs2_error): Trs2_timestamp_domain; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * retrieve timestamp from frame handle in milliseconds
 * \param[in] frame      handle returned from a callback
 * \param[out] error     if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return               the timestamp of the frame in milliseconds
 *)
Trs2_get_frame_timestamp = function(const frame: Prs2_frame;
                                    error: PPrs2_error): Trs2_time; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * retrieve frame parent sensor from frame handle
 * \param[in] frame      handle returned from a callback
 * \param[out] error     if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return               the parent sensor of the frame
 *)
Trs2_get_frame_sensor = function(const frame: Prs2_frame;
                                 error: PPrs2_error): Prs2_sensor; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * retrieve frame number from frame handle
 * \param[in] frame      handle returned from a callback
 * \param[out] error     if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return               the frame nubmer of the frame, in milliseconds since the device was started
 *)
Trs2_get_frame_number = function(const frame: Prs2_frame;
                                 error: PPrs2_error): UInt64; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * retrieve data size from frame handle
 * \param[in] frame      handle returned from a callback
 * \param[out] error     if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return               the size of the frame data
 *)
Trs2_get_frame_data_size = function(const frame: Prs2_frame;
                                    error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * retrieve data from frame handle
 * \param[in] frame      handle returned from a callback
 * \param[out] error     if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return               the pointer to the start of the frame data
 *)
Trs2_get_frame_data = function(const frame: Prs2_frame;
                               error: PPrs2_error): Pointer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * retrieve frame width in pixels
 * \param[in] frame      handle returned from a callback
 * \param[out] error     if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return               frame width in pixels
 *)
Trs2_get_frame_width = function(const frame: Prs2_frame;
                                error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * retrieve frame height in pixels
 * \param[in] frame      handle returned from a callback
 * \param[out] error     if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return               frame height in pixels
 *)
Trs2_get_frame_height = function(const frame: Prs2_frame;
                                 error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * retrieve the scaling factor to use when converting a depth frame's get_data() units to meters
 * \return float - depth, in meters, per 1 unit stored in the frame data
 *)
Trs2_depth_frame_get_units = function(const frame: Prs2_frame;
                                      error: PPrs2_error): Single; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * retrieve frame stride in bytes (number of bytes from start of line N to start of line N+1)
 * \param[in] frame      handle returned from a callback
 * \param[out] error     if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return               stride in bytes
 *)
Trs2_get_frame_stride_in_bytes = function(const frame: Prs2_frame;
                                          error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * retrieve bits per pixels in the frame image
 * (note that bits per pixel is not necessarily divided by 8, as in 12bpp)
 * \param[in] frame      handle returned from a callback
 * \param[out] error     if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return               bits per pixel
 *)
Trs2_get_frame_bits_per_pixel = function(const frame: Prs2_frame;
                                         error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * create additional reference to a frame without duplicating frame data
 * \param[in] frame      handle returned from a callback
 * \param[out] error     if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return               new frame reference, has to be released by rs2_release_frame
 *)
Trs2_frame_add_ref = procedure(frame: Prs2_frame;
                               error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * relases the frame handle
 * \param[in] frame handle returned from a callback
 *)
Trs2_release_frame = procedure(frame: Prs2_frame); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * communicate to the library you intend to keep the frame alive for a while
 * this will remove the frame from the regular count of the frame pool
 * once this function is called, the SDK can no longer guarantee 0-allocations during frame cycling
 * \param[in] frame handle returned from a callback
 *)
Trs2_keep_frame = procedure(frame: Prs2_frame); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * When called on Points frame type, this method returns a pointer to an array of 3D vertices of the model
 * The coordinate system is: X right, Y up, Z away from the camera. Units: Meters
 * \param[in] frame       Points frame
 * \param[out] error      If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return                Pointer to an array of vertices, lifetime is managed by the frame
 *)
Trs2_get_frame_vertices = function(const frame: Prs2_frame;
                                   error: PPrs2_error): Prs2_vertex; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * When called on Points frame type, this method creates a ply file of the model with the given file name.
 * \param[in] frame       Points frame
 * \param[in] fname       The name for the ply file
 * \param[in] texture     Texture frame
 * \param[out] error      If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_export_to_ply = procedure(const frame: Prs2_frame;
                               const fname: PAnsiChar;
                               texture: Prs2_frame;
                               error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * When called on Points frame type, this method returns a pointer to an array of texture coordinates per vertex
 * Each coordinate represent a (u,v) pair within [0,1] range, to be mapped to texture image
 * \param[in] frame       Points frame
 * \param[out] error      If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return                Pointer to an array of texture coordinates, lifetime is managed by the frame
 *)
Trs2_get_frame_texture_coordinates = function(const frame: Prs2_frame;
                                              error: PPrs2_error): Prs2_pixel; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * When called on Points frame type, this method returns the number of vertices in the frame
 * \param[in] frame       Points frame
 * \param[out] error      If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return                Number of vertices
 *)
Trs2_get_frame_points_count = function(const frame: Prs2_frame;
                                       error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Returns the stream profile that was used to start the stream of this frame
 * \param[in] frame       frame reference, owned by the user
 * \param[out] error      If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return                Pointer to the stream profile object, lifetime is managed elsewhere
 *)
Trs2_get_frame_stream_profile = function(const frame: Prs2_frame;
                                         error: PPrs2_error): Prs2_stream_profile; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Test if the given frame can be extended to the requested extension
 * \param[in]  frame             Realsense frame
 * \param[in]  extension_type    The extension to which the frame should be tested if it is extendable
 * \param[out] error             If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return non-zero value iff the frame can be extended to the given extension
 *)
Trs2_is_frame_extendable_to = function(const frame: Prs2_frame;
                                       extension_type: Trs2_extension;
                                       error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Allocate new video frame using a frame-source provided form a processing block
 * \param[in] source      Frame pool to allocate the frame from
 * \param[in] new_stream  New stream profile to assign to newly created frame
 * \param[in] original    A reference frame that can be used to fill in auxilary information like format, width, height, bpp, stride (if applicable)
 * \param[in] new_bpp     New value for bits per pixel for the allocated frame
 * \param[in] new_width   New value for width for the allocated frame
 * \param[in] new_height  New value for height for the allocated frame
 * \param[in] new_stride  New value for stride in bytes for the allocated frame
 * \param[in] frame_type  New value for frame type for the allocated frame
 * \param[out] error      If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return                reference to a newly allocated frame, must be released with release_frame
 *                        memory for the frame is likely to be re-used from previous frame, but in lack of available frames in the pool will be allocated from the free store
 *)
Trs2_allocate_synthetic_video_frame = function(source: Prs2_source;
                                               const new_stream: Prs2_stream_profile;
                                               original: Prs2_frame;
                                               new_bpp: Integer;
                                               new_width: Integer;
                                               new_height: Integer;
                                               new_stride: Integer;
                                               frame_type: Trs2_extension;
                                               error: PPrs2_error): Prs2_frame; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Allocate new motion frame using a frame-source provided form a processing block
 * \param[in] source      Frame pool to allocate the frame from
 * \param[in] new_stream  New stream profile to assign to newly created frame
 * \param[in] original    A reference frame that can be used to fill in auxilary information like format, width, height, bpp, stride (if applicable)
 * \param[in] frame_type  New value for frame type for the allocated frame
 * \param[out] error      If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return                reference to a newly allocated frame, must be released with release_frame
 *                        memory for the frame is likely to be re-used from previous frame, but in lack of available frames in the pool will be allocated from the free store
 *)
Trs2_allocate_synthetic_motion_frame = function(source: Prs2_source;
                                                const new_stream: Prs2_stream_profile;
                                                original: Prs2_frame;
                                                frame_type: Trs2_extension;
                                                error: PPrs2_error): Prs2_frame; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Allocate new points frame using a frame-source provided from a processing block
 * \param[in] source      Frame pool to allocate the frame from
 * \param[in] new_stream  New stream profile to assign to newly created frame
 * \param[in] original    A reference frame that can be used to fill in auxilary information like format, width, height, bpp, stride (if applicable)
 * \param[out] error      If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return                reference to a newly allocated frame, must be released with release_frame
 *                        memory for the frame is likely to be re-used from previous frame, but in lack of available frames in the pool will be allocated from the free store
 *)
Trs2_allocate_points = function(source: Prs2_source;
                                const new_stream: Prs2_stream_profile;
                                original: Prs2_frame;
                                error: PPrs2_error): Prs2_frame; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Allocate new composite frame, aggregating a set of existing frames
 * \param[in] source      Frame pool to allocate the frame from
 * \param[in] frames      Array of existing frames
 * \param[in] count       Number of input frames
 * \param[out] error      If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return                reference to a newly allocated frame, must be released with release_frame
 *                        when composite frame gets released it will automatically release all of the input frames
 *)
Trs2_allocate_composite_frame = function(source: Prs2_source;
                                         frames: PPrs2_frame;
                                         count: Integer;
                                         error: PPrs2_error): Prs2_frame; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Extract frame from within a composite frame
 * \param[in] composite   Composite frame
 * \param[in] index       Index of the frame to extract within the composite frame
 * \param[out] error      If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return                returns reference to a frame existing within the composite frame
 *                        If you wish to keep this frame after the composite is released, you need to call acquire_ref
 *                        Otherwise the resulting frame lifetime is bound by owning composite frame
 *)
Trs2_extract_frame = function(composite: Prs2_frame;
                              index: Integer;
                              error: PPrs2_error): Prs2_frame; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Get number of frames embedded within a composite frame
 * \param[in] composite   Composite input frame
 * \param[out] error      If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return                Number of embedded frames
 *)
Trs2_embedded_frames_count = function(composite: Prs2_frame;
                                      error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * This method will dispatch frame callback on a frame
 * \param[in] source      Frame pool provided by the processing block
 * \param[in] frame       Frame to dispatch, frame ownership is passed to this function, so you don't have to call release_frame after it
 * \param[out] error      If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_synthetic_frame_ready = procedure(source: Prs2_source;
                                       frame: Prs2_frame;
                                       error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};


(***
 * When called on Pose frame type, this method returns the transformation represented by the pose data
 * \param[in] frame       Pose frame
 * \param[out] pose       Pointer to a user allocated struct, which contains the pose info after a successful return
 * \param[out] error      If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_pose_frame_get_pose_data = procedure(const frame: Prs2_frame;
                                          pose: Prs2_pose;
                                          error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};


//------------------------------------------------------------------------------

type
(*** \brief Defines general configuration controls.
 These can generally be mapped to camera UVC controls, and can be set / queried at any time unless stated otherwise.
 *)
Trs2_option = (
  RS2_OPTION_BACKLIGHT_COMPENSATION, (***< Enable / disable color backlight compensation*)
  RS2_OPTION_BRIGHTNESS, (***< Color image brightness*)
  RS2_OPTION_CONTRAST, (***< Color image contrast*)
  RS2_OPTION_EXPOSURE, (***< Controls exposure time of color camera. Setting any value will disable auto exposure*)
  RS2_OPTION_GAIN, (***< Color image gain*)
  RS2_OPTION_GAMMA, (***< Color image gamma setting*)
  RS2_OPTION_HUE, (***< Color image hue*)
  RS2_OPTION_SATURATION, (***< Color image saturation setting*)
  RS2_OPTION_SHARPNESS, (***< Color image sharpness setting*)
  RS2_OPTION_WHITE_BALANCE, (***< Controls white balance of color image. Setting any value will disable auto white balance*)
  RS2_OPTION_ENABLE_AUTO_EXPOSURE, (***< Enable / disable color image auto-exposure*)
  RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, (***< Enable / disable color image auto-white-balance*)
  RS2_OPTION_VISUAL_PRESET, (***< Provide access to several recommend sets of option presets for the depth camera *)
  RS2_OPTION_LASER_POWER, (***< Power of the laser emitter, with 0 meaning projector off*)
  RS2_OPTION_ACCURACY, (***< Set the number of patterns projected per frame. The higher the accuracy value the more patterns projected. Increasing the number of patterns help to achieve better accuracy. Note that this control is affecting the Depth FPS *)
  RS2_OPTION_MOTION_RANGE, (***< Motion vs. Range trade-off, with lower values allowing for better motion sensitivity and higher values allowing for better depth range*)
  RS2_OPTION_FILTER_OPTION, (***< Set the filter to apply to each depth frame. Each one of the filter is optimized per the application requirements*)
  RS2_OPTION_CONFIDENCE_THRESHOLD, (***< The confidence level threshold used by the Depth algorithm pipe to set whether a pixel will get a valid range or will be marked with invalid range*)
  RS2_OPTION_EMITTER_ENABLED, (***< Emitter select: 0 – disable all emitters. 1 – enable laser. 2 – enable auto laser. 3 – enable LED.*)
  RS2_OPTION_FRAMES_QUEUE_SIZE, (***< Number of frames the user is allowed to keep per stream. Trying to hold-on to more frames will cause frame-drops.*)
  RS2_OPTION_TOTAL_FRAME_DROPS, (***< Total number of detected frame drops from all streams *)
  RS2_OPTION_AUTO_EXPOSURE_MODE, (***< Auto-Exposure modes: Static, Anti-Flicker and Hybrid *)
  RS2_OPTION_POWER_LINE_FREQUENCY, (***< Power Line Frequency control for anti-flickering Off/50Hz/60Hz/Auto *)
  RS2_OPTION_ASIC_TEMPERATURE, (***< Current Asic Temperature *)
  RS2_OPTION_ERROR_POLLING_ENABLED, (***< disable error handling *)
  RS2_OPTION_PROJECTOR_TEMPERATURE, (***< Current Projector Temperature *)
  RS2_OPTION_OUTPUT_TRIGGER_ENABLED, (***< Enable / disable trigger to be outputed from the camera to any external device on every depth frame *)
  RS2_OPTION_MOTION_MODULE_TEMPERATURE, (***< Current Motion-Module Temperature *)
  RS2_OPTION_DEPTH_UNITS, (***< Number of meters represented by a single depth unit *)
  RS2_OPTION_ENABLE_MOTION_CORRECTION, (***< Enable/Disable automatic correction of the motion data *)
  RS2_OPTION_AUTO_EXPOSURE_PRIORITY, (***< Allows sensor to dynamically ajust the frame rate depending on lighting conditions *)
  RS2_OPTION_COLOR_SCHEME, (***< Color scheme for data visualization *)
  RS2_OPTION_HISTOGRAM_EQUALIZATION_ENABLED, (***< Perform histogram equalization post-processing on the depth data *)
  RS2_OPTION_MIN_DISTANCE, (***< Minimal distance to the target *)
  RS2_OPTION_MAX_DISTANCE, (***< Maximum distance to the target *)
  RS2_OPTION_TEXTURE_SOURCE, (***< Texture mapping stream unique ID *)
  RS2_OPTION_FILTER_MAGNITUDE, (***< The 2D-filter effect. The specific interpretation is given within the context of the filter *)
  RS2_OPTION_FILTER_SMOOTH_ALPHA, (***< 2D-filter parameter controls the weight/radius for smoothing.*)
  RS2_OPTION_FILTER_SMOOTH_DELTA, (***< 2D-filter range/validity threshold*)
  RS2_OPTION_HOLES_FILL, (***< Enhance depth data post-processing with holes filling where appropriate*)
  RS2_OPTION_STEREO_BASELINE, (***< The distance in mm between the first and the second imagers in stereo-based depth cameras*)
  RS2_OPTION_AUTO_EXPOSURE_CONVERGE_STEP, (***< Allows dynamically ajust the converge step value of the target exposure in Auto-Exposure algorithm*)
  RS2_OPTION_INTER_CAM_SYNC_MODE, (***< Impose Inter-camera HW synchronization mode. Applicable for D400/L500/Rolling Shutter SKUs *)
  RS2_OPTION_STREAM_FILTER, (***< Select a stream to process *)
  RS2_OPTION_STREAM_FORMAT_FILTER, (***< Select a stream format to process *)
  RS2_OPTION_STREAM_INDEX_FILTER, (***< Select a stream index to process *)
  RS2_OPTION_EMITTER_ON_OFF, (***< When supported, this option make the camera to switch the emitter state every frame. 0 for disabled, 1 for enabled *)
  RS2_OPTION_ZERO_ORDER_POINT_X, (***< Zero order point x*)
  RS2_OPTION_ZERO_ORDER_POINT_Y, (***< Zero order point y*)
  RS2_OPTION_LLD_TEMPERATURE, (***< LLD temperature*)
  RS2_OPTION_MC_TEMPERATURE, (***< MC temperature*)
  RS2_OPTION_MA_TEMPERATURE, (***< MA temperature*)
  RS2_OPTION_HARDWARE_PRESET, (***< Hardware stream configuration *)
  RS2_OPTION_GLOBAL_TIME_ENABLED, (***< disable global time  *)
  RS2_OPTION_APD_TEMPERATURE, (***< APD temperature*)
  RS2_OPTION_ENABLE_MAPPING, (***< Enable an internal map *)
  RS2_OPTION_ENABLE_RELOCALIZATION, (***< Enable appearance based relocalization *)
  RS2_OPTION_ENABLE_POSE_JUMPING, (***< Enable position jumping *)
  RS2_OPTION_ENABLE_DYNAMIC_CALIBRATION, (***< Enable dynamic calibration *)
  RS2_OPTION_DEPTH_OFFSET, (***< Offset from sensor to depth origin in millimetrers*)
  RS2_OPTION_LED_POWER, (***< Power of the LED (light emitting diode), with 0 meaning LED off*)
  RS2_OPTION_ZERO_ORDER_ENABLED, (***< Toggle Zero-Order mode *)
  RS2_OPTION_ENABLE_MAP_PRESERVATION, (***< Preserve previous map when starting *)
  RS2_OPTION_FREEFALL_DETECTION_ENABLED, (***< Enable/disable sensor shutdown when a free-fall is detected (on by default) *)
  RS2_OPTION_AVALANCHE_PHOTO_DIODE, (***< Changes the exposure time of Avalanche Photo Diode in the receiver *)
  RS2_OPTION_POST_PROCESSING_SHARPENING,  (***< Changes the amount of sharpening in the post-processed image *)
  RS2_OPTION_PRE_PROCESSING_SHARPENING, (***< Changes the amount of sharpening in the pre-processed image *)
  RS2_OPTION_NOISE_FILTERING, (***< Control edges and background noise *)
  RS2_OPTION_INVALIDATION_BYPASS, (***< Enable\disable pixel invalidation *)
  RS2_OPTION_AMBIENT_LIGHT, (***< Change the depth ambient light see rs2_ambient_light for values *)
  RS2_OPTION_SENSOR_MODE, (***< The resolution mode: see rs2_sensor_mode for values *)
  RS2_OPTION_EMITTER_ALWAYS_ON, (***< Enable Laser On constantly (GS SKU Only) *)
  RS2_OPTION_THERMAL_COMPENSATION, (***< Depth Thermal Compensation for selected D400 SKUs *)
  RS2_OPTION_TRIGGER_CAMERA_ACCURACY_HEALTH, (***< Enable depth & color frame sync with periodic calibration for proper alignment *)
  RS2_OPTION_RESET_CAMERA_ACCURACY_HEALTH,
  RS2_OPTION_HOST_PERFORMANCE, (***< Set host performance mode to optimize device settings so host can keep up with workload, for example, USB transaction granularity, setting option to low performance host leads to larger USB transaction size and reduced number of transactions which improves performance and stability if host is relatively weak as compared to workload *)
  RS2_OPTION_HDR_ENABLED,  (***< Enable / disable HDR *)
  RS2_OPTION_SEQUENCE_NAME, (***< HDR Sequence size *)
  RS2_OPTION_SEQUENCE_SIZE, (***< HDR Sequence size *)
  RS2_OPTION_SEQUENCE_ID, (***< HDR Sequence ID - 0 is not HDR; sequence ID for HDR configuartion starts from 1 *)
  RS2_OPTION_COUNT (***< Number of enumeration values. Not a valid input: intended to be used in for-loops. *)
);

// This function is being deprecated. For existing options it will return option name, but for future API additions the user should call rs2_get_option_name instead.
Trs2_option_to_string = function(option: Trs2_option): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

type
(*** \brief For SR300 devices: provides optimized settings (presets) for specific types of usage. *)
Trs2_sr300_visual_preset = (
  RS2_SR300_VISUAL_PRESET_SHORT_RANGE, (***< Preset for short range *)
  RS2_SR300_VISUAL_PRESET_LONG_RANGE, (***< Preset for long range *)
  RS2_SR300_VISUAL_PRESET_BACKGROUND_SEGMENTATION, (***< Preset for background segmentation *)
  RS2_SR300_VISUAL_PRESET_GESTURE_RECOGNITION, (***< Preset for gesture recognition *)
  RS2_SR300_VISUAL_PRESET_OBJECT_SCANNING, (***< Preset for object scanning *)
  RS2_SR300_VISUAL_PRESET_FACE_ANALYTICS, (***< Preset for face analytics *)
  RS2_SR300_VISUAL_PRESET_FACE_LOGIN, (***< Preset for face login *)
  RS2_SR300_VISUAL_PRESET_GR_CURSOR, (***< Preset for GR cursor *)
  RS2_SR300_VISUAL_PRESET_DEFAULT, (***< Camera default settings *)
  RS2_SR300_VISUAL_PRESET_MID_RANGE, (***< Preset for mid-range *)
  RS2_SR300_VISUAL_PRESET_IR_ONLY, (***< Preset for IR only *)
  RS2_SR300_VISUAL_PRESET_COUNT                       (***< Number of enumeration values. Not a valid input: intended to be used in for-loops. *)
);

Trs2_sr300_visual_preset_to_string = function(preset: Trs2_sr300_visual_preset): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

type
(*** \brief For RS400 devices: provides optimized settings (presets) for specific types of usage. *)
Trs2_rs400_visual_preset = (
  RS2_RS400_VISUAL_PRESET_CUSTOM,
  RS2_RS400_VISUAL_PRESET_DEFAULT,
  RS2_RS400_VISUAL_PRESET_HAND,
  RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY,
  RS2_RS400_VISUAL_PRESET_HIGH_DENSITY,
  RS2_RS400_VISUAL_PRESET_MEDIUM_DENSITY,
  RS2_RS400_VISUAL_PRESET_REMOVE_IR_PATTERN,
  RS2_RS400_VISUAL_PRESET_COUNT (***< Number of enumeration values. Not a valid input: intended to be used in for-loops. *)
);

Trs2_rs400_visual_preset_to_string = function(preset: Trs2_rs400_visual_preset): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

type
(*** \brief For L500 devices: provides optimized settings (presets) for specific types of usage. *)
Trs2_l500_visual_preset = (
  RS2_L500_VISUAL_PRESET_CUSTOM,
  RS2_L500_VISUAL_PRESET_DEFAULT,
  RS2_L500_VISUAL_PRESET_NO_AMBIENT,
  RS2_L500_VISUAL_PRESET_LOW_AMBIENT,
  RS2_L500_VISUAL_PRESET_MAX_RANGE,
  RS2_L500_VISUAL_PRESET_SHORT_RANGE,
  RS2_L500_VISUAL_PRESET_COUNT (***< Number of enumeration values. Not a valid input: intended to be used in for-loops. *)
);

Trs2_l500_visual_preset_to_string = function(preset: Trs2_l500_visual_preset): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

type
(*** \brief For setting the camera_mode option *)
Trs2_sensor_mode = (
  RS2_SENSOR_MODE_VGA,
  RS2_SENSOR_MODE_XGA,
  RS2_SENSOR_MODE_QVGA,
  RS2_SENSOR_MODE_COUNT (***< Number of enumeration values. Not a valid input: intended to be used in for-loops. *)
);

Trs2_sensor_mode_to_string = function(preset: Trs2_sensor_mode): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

type
(*** \brief ambient light for RS2_OPTION_AMBIENT_LIGHT option. *)
Trs2_ambient_light = (
  RS2_AMBIENT_LIGHT_NO_AMBIENT = 1,
  RS2_AMBIENT_LIGHT_LOW_AMBIENT = 2
);

Trs2_ambient_light_to_string = function(preset: Trs2_ambient_light): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

type
(*** \brief values for RS2_OPTION_TRIGGER_CAMERA_ACCURACY_HEALTH option. *)
Trs2_cah_trigger = (
  RS2_CAH_TRIGGER_MANUAL = 0,  (***< not triggered until you give _NOW *)
  RS2_CAH_TRIGGER_NOW    = 1,  (***< triggers CAH and leaves previous value intact! *)
  RS2_CAH_TRIGGER_AUTO   = 2,  (***< triggered periodically or with certain conditions *)
  RS2_CAH_TRIGGER_COUNT    (***< Number of enumeration values. Not a valid input: intended to be used in for-loops. *)
);

Trs2_cah_trigger_to_string = function(preset: Trs2_cah_trigger): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

type
(*** \brief values for RS2_OPTION_HOST_PERFORMANCE option. *)
Trs2_host_perf_mode = (
  RS2_HOST_PERF_DEFAULT = 0,  (***< no change in settings, use device defaults *)
  RS2_HOST_PERF_LOW = 1,  (***< low performance host mode, if host cannot keep up with workload, this option may improve stability, for example, it sets larger USB transaction granularity, reduces number of transactions and improve performance and stability on relatively weak hosts as compared to the workload *)
  RS2_HOST_PERF_HIGH = 2, (***< high performance host mode, if host is strong as compared to the work and can handle workload without delay, this option sets smaller USB transactions granularity and as result larger number of transactions and workload on host, but reduces chance in device frame drops *)
  RS2_HOST_PERF_COUNT     (***< Number of enumeration values. Not a valid input: intended to be used in for-loops. *)
);

Trs2_host_perf_mode_to_string = function(perf: Trs2_host_perf_mode): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * check if an option is read-only
 * \param[in] options  the options container
 * \param[in] option   option id to be checked
 * \param[out] error   if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return true if option is read-only
 *)
Trs2_is_option_read_only = function(const options: Prs2_options;
                                    option: Trs2_option;
                                    error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * read option value from the sensor
 * \param[in] options  the options container
 * \param[in] option   option id to be queried
 * \param[out] error   if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return value of the option
 *)
Trs2_get_option = function(const options: Prs2_options;
                           option: Trs2_option;
                           error: PPrs2_error): Single; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * write new value to sensor option
 * \param[in] options    the options container
 * \param[in] option     option id to be queried
 * \param[in] value      new value for the option
 * \param[out] error     if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_set_option = procedure(const options: Prs2_options;
                            option: Trs2_option;
                            value: Single;
                            error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * get the list of supported options of options container
 * \param[in] options    the options container
 * \param[out] error     if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_get_options_list = function(const options: Prs2_options;
                                 error: PPrs2_error): Prs2_options_list; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * get the size of options list
 * \param[in] options    the option list
 * \param[out] error     if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_get_options_list_size = function(const options: Prs2_options_list;
                                      error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * get option name
 * \param[in] options    the options container
 * \param[in] option     option id to be checked
 * \param[out] error     if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return human-readable option name
 *)
Trs2_get_option_name = function(const options: Prs2_options;
                                option: Trs2_option;
                                error: PPrs2_error): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * get the specific option from options list
 * \param[in] i    the index of the option
 * \param[out] error     if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_get_option_from_list = function(const options: Prs2_options_list;
                                     i: Integer;
                                     error: PPrs2_error): Trs2_option; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Deletes options list
 * \param[in] list list to delete
 *)
Trs2_delete_options_list = procedure(list: Prs2_options_list); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * check if particular option is supported by a subdevice
 * \param[in] options    the options container
 * \param[in] option     option id to be checked
 * \param[out] error     if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return true if option is supported
 *)
Trs2_supports_option = function(const options: Prs2_options;
                                option: Trs2_option;
                                error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * retrieve the available range of values of a supported option
 * \param[in] sensor  the RealSense device
 * \param[in] option  the option whose range should be queried
 * \param[out] min    the minimum value which will be accepted for this option
 * \param[out] max    the maximum value which will be accepted for this option
 * \param[out] step   the granularity of options which accept discrete values, or zero if the option accepts continuous values
 * \param[out] def    the default value of the option
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_get_option_range = procedure(const sensor: Prs2_options;
                                  option: Trs2_option;
                                  min: PSingle;
                                  max: PSingle;
                                  step: PSingle;
                                  def: PSingle;
                                  error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * get option description
 * \param[in] options    the options container
 * \param[in] option     option id to be checked
 * \param[out] error     if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return human-readable option description
 *)
Trs2_get_option_description = function(const options: Prs2_options;
                                       option: Trs2_option;
                                       error: PPrs2_error): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * get option value description (in case specific option value hold special meaning)
 * \param[in] options    the options container
 * \param[in] option     option id to be checked
 * \param[in] value      value of the option
 * \param[out] error     if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return human-readable description of a specific value of an option or null if no special meaning
 *)
Trs2_get_option_value_description = function(const options: Prs2_options;
                                             option: Trs2_option;
                                             value: Single;
                                             error: PPrs2_error): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};


//------------------------------------------------------------------------------

(***
 * Creates Depth-Colorizer processing block that can be used to quickly visualize the depth data
 * This block will accept depth frames as input and replace them by depth frames with format RGB8
 * Non-depth frames are passed through
 * Further customization will be added soon (format, color-map, histogram equalization control)
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_create_colorizer = function(error: PPrs2_error): Prs2_processing_block; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Creates Sync processing block. This block accepts arbitrary frames and output composite frames of best matches
 * Some frames may be released within the syncer if they are waiting for match for too long
 * Syncronization is done (mostly) based on timestamps so good hardware timestamps are a pre-condition
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_create_sync_processing_block = function(error: PPrs2_error): Prs2_processing_block; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Creates Point-Cloud processing block. This block accepts depth frames and outputs Points frames
 * In addition, given non-depth frame, the block will align texture coordinate to the non-depth stream
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_create_pointcloud = function(error: PPrs2_error): Prs2_processing_block; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Creates YUY decoder processing block. This block accepts raw YUY frames and outputs frames of other formats.
 * YUY is a common video format used by a variety of web-cams. It benefits from packing pixels into 2 bytes per pixel
 * without signficant quality drop. YUY representation can be converted back to more usable RGB form,
 * but this requires somewhat costly conversion.
 * The SDK will automatically try to use SSE2 and AVX instructions and CUDA where available to get
 * best performance. Other implementations (using GLSL, OpenCL, Neon and NCS) should follow.
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_create_yuy_decoder = function(error: PPrs2_error): Prs2_processing_block; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Creates depth thresholding processing block
 * By controlling min and max options on the block, one could filter out depth values
 * that are either too large or too small, as a software post-processing step
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_create_threshold = function(error: PPrs2_error): Prs2_processing_block; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Creates depth units transformation processing block
 * All of the pixels are transformed from depth units into meters.
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_create_units_transform = function(error: PPrs2_error): Prs2_processing_block; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * This method creates new custom processing block. This lets the users pass frames between module boundaries for processing
 * This is an infrastructure function aimed at middleware developers, and also used by provided blocks such as sync, colorizer, etc..
 * \param proc       Processing function to be applied to every frame entering the block
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return           new processing block, to be released by rs2_delete_processing_block
 *)
Trs2_create_processing_block = function(proc: Prs2_frame_processor_callback;
                                        error: PPrs2_error): Prs2_processing_block; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * This method creates new custom processing block from function pointer. This lets the users pass frames between module boundaries for processing
 * This is an infrastructure function aimed at middleware developers, and also used by provided blocks such as sync, colorizer, etc..
 * \param proc       Processing function pointer to be applied to every frame entering the block
 * \param context    User context (can be anything or null) to be passed later as ctx param of the callback
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return           new processing block, to be released by rs2_delete_processing_block
 *)
Trs2_create_processing_block_fptr = function(//proc: rs2_frame_processor_callback_ptr;
                                             context: Pointer;
                                             error: PPrs2_error): Prs2_processing_block; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * This method adds a custom option to a custom processing block. This is a simple float that can be accessed via rs2_set_option and rs2_get_option
 * This is an infrastructure function aimed at middleware developers, and also used by provided blocks such as save_to_ply, etc..
 * \param[in] block      Processing block
 * \param[in] option_id  an int ID for referencing the option
 * \param[in] min     the minimum value which will be accepted for this option
 * \param[in] max     the maximum value which will be accepted for this option
 * \param[in] step    the granularity of options which accept discrete values, or zero if the option accepts continuous values
 * \param[in] def     the default value of the option. This will be the initial value.
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return            true if adding the option succeeds. false if it fails e.g. an option with this id is already registered
 *)
Trs2_processing_block_register_simple_option = function(block: Prs2_processing_block;
                                                        option_id: Trs2_option;
                                                        min: Single;
                                                        max: Single;
                                                        step: Single;
                                                        def: Single;
                                                        error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * This method is used to direct the output from the processing block to some callback or sink object
 * \param[in] block          Processing block
 * \param[in] on_frame       Callback to be invoked every time the processing block calls frame_ready
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_start_processing = procedure(block: Prs2_processing_block;
                                  on_frame: Prs2_frame_callback;
                                  error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * This method is used to direct the output from the processing block to some callback or sink object
 * \param[in] block          Processing block
 * \param[in] on_frame       Callback function to be invoked every time the processing block calls frame_ready
 * \param[in] user           User context for the callback (can be anything or null)
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_start_processing_fptr = procedure(block: Prs2_processing_block;
                                       on_frame: Prs2_frame_callback;
                                       user: Pointer;
                                       error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * This method is used to direct the output from the processing block to a dedicated queue object
 * \param[in] block          Processing block
 * \param[in] queue          Queue to place the processed frames to
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_start_processing_queue = procedure(block: Prs2_processing_block;
                                        queue: Prs2_frame_queue;
                                        error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * This method is used to pass frame into a processing block
 * \param[in] block          Processing block
 * \param[in] frame          Frame to process, ownership is moved to the block object
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_process_frame = procedure(block: Prs2_processing_block;
                               frame: Prs2_frame;
                               error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Deletes the processing block
 * \param[in] block          Processing block
 *)
Trs2_delete_processing_block = procedure(block: Prs2_processing_block); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * create frame queue. frame queues are the simplest x-platform synchronization primitive provided by librealsense
 * to help developers who are not using async APIs
 * \param[in] capacity max number of frames to allow to be stored in the queue before older frames will start to get dropped
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return handle to the frame queue, must be released using rs2_delete_frame_queue
 *)
Trs2_create_frame_queue = function(capacity: Integer;
                                   error: PPrs2_error): Prs2_frame_queue; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * deletes frame queue and releases all frames inside it
 * \param[in] queue queue to delete
 *)
Trs2_delete_frame_queue = procedure(queue: Prs2_frame_queue); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * wait until new frame becomes available in the queue and dequeue it
 * \param[in] queue the frame queue data structure
 * \param[in] timeout_ms   max time in milliseconds to wait until an exception will be thrown
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return frame handle to be released using rs2_release_frame
 *)
Trs2_wait_for_frame = function(queue: Prs2_frame_queue;
                               timeout_ms: FixedUInt;
                               error: PPrs2_error): Prs2_frame; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * poll if a new frame is available and dequeue if it is
 * \param[in] queue the frame queue data structure
 * \param[out] output_frame frame handle to be released using rs2_release_frame
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return true if new frame was stored to output_frame
 *)
Trs2_poll_for_frame = function(queue: Prs2_frame_queue;
                               output_frame: PPrs2_frame;
                               error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * wait until new frame becomes available in the queue and dequeue it
 * \param[in] queue          the frame queue data structure
 * \param[in] timeout_ms     max time in milliseconds to wait until a frame becomes available
 * \param[out] output_frame  frame handle to be released using rs2_release_frame
 * \param[out] error         if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return true if new frame was stored to output_frame
 *)
Trs2_try_wait_for_frame = function(queue: Prs2_frame_queue;
                                   timeout_ms: FixedUInt;
                                   output_frame: PPrs2_frame;
                                   error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * enqueue new frame into a queue
 * \param[in] frame frame handle to enqueue (this operation passed ownership to the queue)
 * \param[in] queue the frame queue data structure
 *)
Trs2_enqueue_frame = procedure(frame: Prs2_frame;
                               queue: Pointer); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Creates Align processing block.
 * \param[in] align_to   stream type to be used as the target of frameset alignment
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_create_align = function(//align_to: Trs2_stream;
                             error: PPrs2_error): Prs2_processing_block; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Creates Depth post-processing filter block. This block accepts depth frames, applies decimation filter and plots modified prames
 * Note that due to the modifiedframe size, the decimated frame repaces the original one
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_create_decimation_filter_block = function(error: PPrs2_error): Prs2_processing_block; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Creates Depth post-processing filter block. This block accepts depth frames, applies temporal filter
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_create_temporal_filter_block = function(error: PPrs2_error): Prs2_processing_block; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Creates Depth post-processing spatial filter block. This block accepts depth frames, applies spatial filters and plots modified prames
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_create_spatial_filter_block = function(error: PPrs2_error): Prs2_processing_block; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Creates a post processing block that provides for depth<->disparity domain transformation for stereo-based depth modules
 * \param[in] transform_to_disparity flag select the transform direction:  true = depth->disparity, and vice versa
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_create_disparity_transform_block = function(transform_to_disparity: Byte;
                                                 error: PPrs2_error): Prs2_processing_block; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Creates Depth post-processing hole filling block. The filter replaces empty pixels with data from adjacent pixels based on the method selected
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_create_hole_filling_filter_block = function(error: PPrs2_error): Prs2_processing_block; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Creates a rates printer block. The printer prints the actual FPS of the invoked frame stream.
 * The block ignores reapiting frames and calculats the FPS only if the frame number of the relevant frame was changed.
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_create_rates_printer_block = function(error: PPrs2_error):Prs2_processing_block; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Creates Depth post-processing zero order fix block. The filter invalidates pixels that has a wrong value due to zero order effect
 * \param[out] error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return               zero order fix processing block
 *)
Trs2_create_zero_order_invalidation_block = function(error: PPrs2_error): Prs2_processing_block; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Creates Depth frame decompression module. Decoded frames compressed and transmitted with Z16H variable-lenght Huffman code to
 * standartized Z16 Depth data format. Using the compression allows to reduce the Depth frames bandwidth by more than 50 percent
 * \param[out] error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return               Huffman-code decompression processing block
 *)
Trs2_create_huffman_depth_decompress_block = function(error: PPrs2_error): Prs2_processing_block; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Creates a hdr_merge processing block.
 * The block merges between two depth frames with different exposure values
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_create_hdr_merge_processing_block = function(error: PPrs2_error): Prs2_processing_block; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Creates a sequence_id_filter processing block.
 * The block lets frames with the selected sequence id pass and blocks frames with other values
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_create_sequence_id_filter = function(error: PPrs2_error): Prs2_processing_block; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Retrieve processing block specific information, like name.
 * \param[in]  block     The processing block
 * \param[in]  info      processing block info type to retrieve
 * \param[out] error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return               The requested processing block info string, in a format specific to the device model
 *)
Trs2_get_processing_block_info = function(const block: Prs2_processing_block;
                                          info: Trs2_camera_info;
                                          error: PPrs2_error): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Check if a processing block supports a specific info type.
 * \param[in]  block     The processing block to check
 * \param[in]  info      The parameter to check for support
 * \param[out] error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return               True if the parameter both exist and well-defined for the specific device
 *)
Trs2_supports_processing_block_info = function(const block: Prs2_processing_block;
                                               info: Trs2_camera_info;
                                               error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Test if the given processing block can be extended to the requested extension
 * \param[in] block processing block
 * \param[in] extension The extension to which the sensor should be tested if it is extendable
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return non-zero value iff the processing block can be extended to the given extension
 *)
Trs2_is_processing_block_extendable_to = function(const block: Prs2_processing_block;
                                                  extension_type: Trs2_extension;
                                                  error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};


//------------------------------------------------------------------------------
type

Trs2_playback_status = (
  RS2_PLAYBACK_STATUS_UNKNOWN, (***< Unknown state *)
  RS2_PLAYBACK_STATUS_PLAYING, (***< One or more sensors were started, playback is reading and raising data *)
  RS2_PLAYBACK_STATUS_PAUSED,  (***< One or more sensors were started, but playback paused reading and paused raising data*)
  RS2_PLAYBACK_STATUS_STOPPED, (***< All sensors were stopped, or playback has ended (all data was read). This is the initial playback status*)
  RS2_PLAYBACK_STATUS_COUNT
);

Trs2_playback_status_to_string = function(status: Trs2_playback_status): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

//typedef void (*rs2_playback_status_changed_callback_ptr)(rs2_playback_status);

(***
 * Creates a recording device to record the given device and save it to the given file
 * \param[in]  device    The device to record
 * \param[in]  file      The desired path to which the recorder should save the data
 * \param[out] error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return A pointer to a device that records its data to file, or null in case of failure
 *)
Trs2_create_record_device = function(const device: Prs2_device;
                                     const _file: PAnsiChar;
                                     error: PPrs2_error): Prs2_device; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
* Creates a recording device to record the given device and save it to the given file
* \param[in]  device                The device to record
* \param[in]  file                  The desired path to which the recorder should save the data
* \param[in]  compression_enabled   Indicates if compression is enabled, 0 means false, otherwise true
* \param[out] error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
* \return A pointer to a device that records its data to file, or null in case of failure
*)
Trs2_create_record_device_ex = function(const device: Prs2_device;
                                        const _file: PAnsiChar;
                                        compression_enabled: Integer;
                                        error: PPrs2_error): Prs2_device; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
* Pause the recording device without stopping the actual device from streaming.
* Pausing will cause the device to stop writing new data to the file, in particular, frames and changes to extensions
* \param[in]  device    A recording device
* \param[out] error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
*)
Trs2_record_device_pause = procedure(const device: Prs2_device;
                                     error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
* Unpause the recording device. Resume will cause the device to continue writing new data to the file, in particular, frames and changes to extensions
* \param[in]  device    A recording device
* \param[out] error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
*)
Trs2_record_device_resume = procedure(const device: Prs2_device;
                                      error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
* Gets the name of the file to which the recorder is writing
* \param[in]  device    A recording device
* \param[out] error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
* \return The  name of the file to which the recorder is writing
*)
Trs2_record_device_filename = function(const device: Prs2_device;
                                       error: PPrs2_error): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
* Creates a playback device to play the content of the given file
* \param[in]  file      Path to the file to play
* \param[out] error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
* \return A pointer to a device that plays data from the file, or null in case of failure
*)
Trs2_create_playback_device = function(_file: PAnsiChar;
                                       error: PPrs2_error): Prs2_device; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Gets the path of the file used by the playback device
 * \param[in] device A playback device
 * \param[out] error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return Path to the file used by the playback device
 *)
Trs2_playback_device_get_file_path = function(const device: Prs2_device;
                                              error: PPrs2_error): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Gets the total duration of the file in units of nanoseconds
 * \param[in] device     A playback device
 * \param[out] error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return Total duration of the file in units of nanoseconds
 *)
Trs2_playback_get_duration = function(const device: Prs2_device;
                                      error: PPrs2_error): UInt64; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Set the playback to a specified time point of the played data
 * \param[in] device     A playback device.
 * \param[in] time       The time point to which playback should seek, expressed in units of nanoseconds (zero value = start)
 * \param[out] error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_playback_seek = procedure(const device: Prs2_device;
                               time: UInt64;
                               error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Gets the current position of the playback in the file in terms of time. Units are expressed in nanoseconds
 * \param[in] device     A playback device
 * \param[out] error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return Current position of the playback in the file in terms of time. Units are expressed in nanoseconds
 *)
Trs2_playback_get_position = function(const device: Prs2_device;
                                      error: PPrs2_error): UInt64; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Pauses the playback
 * Calling pause() in "Paused" status does nothing
 * If pause() is called while playback status is "Playing" or "Stopped", the playback will not play until resume() is called
 * \param[in] device A playback device
 * \param[out] error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_playback_device_resume = procedure(const device: Prs2_device;
                                        error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Un-pauses the playback
 * Calling resume() while playback status is "Playing" or "Stopped" does nothing
 * \param[in] device A playback device
 * \param[out] error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_playback_device_pause = procedure(const device: Prs2_device;
                                       error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Set the playback to work in real time or non real time
 *
 * In real time mode, playback will play the same way the file was recorded.
 * In real time mode if the application takes too long to handle the callback, frames may be dropped.
 * In non real time mode, playback will wait for each callback to finish handling the data before
 * reading the next frame. In this mode no frames will be dropped, and the application controls the
 * frame rate of the playback (according to the callback handler duration).
 * \param[in] device A playback device
 * \param[in] real_time  Indicates if real time is requested, 0 means false, otherwise true
 * \param[out] error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_playback_device_set_real_time = procedure(const device: Prs2_device;
                                               real_time: Integer;
                                               error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Indicates if playback is in real time mode or non real time
 * \param[in] device A playback device
 * \param[out] error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return True iff playback is in real time mode. 0 means false, otherwise true
 *)
Trs2_playback_device_is_real_time = function(const device: Prs2_device;
                                             error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Register to receive callback from playback device upon its status changes
 *
 * Callbacks are invoked from the reading thread, any heavy processing in the callback handler will affect
 * the reading thread and may cause frame drops\ high latency
 * \param[in] device     A playback device
 * \param[in] callback   A callback handler that will be invoked when the playback status changes
 * \param[out] error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_playback_device_set_status_changed_callback = procedure(const device: Prs2_device;
                                                             callback: Prs2_playback_status_changed_callback;
                                                             error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Returns the current state of the playback device
 * \param[in] device     A playback device
 * \param[out] error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return Current state of the playback
 *)
Trs2_playback_device_get_current_status = function(const device: Prs2_device;
                                                   error: PPrs2_error): Trs2_playback_status; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Set the playing speed
 *
 * \param[in] device A playback device
 * \param[in] speed  Indicates a multiplication of the speed to play (e.g: 1 = normal, 0.5 twice as slow)
 * \param[out] error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_playback_device_set_playback_speed = procedure(const device: Prs2_device;
                                                    speed: Single;
                                                    error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(***
 * Stops the playback
 * Calling stop() will stop all streaming playbakc sensors and will reset the playback (returning to beginning of file)
 * \param[in] device A playback device
 * \param[out] error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_playback_device_stop = procedure(const device: Prs2_device;
                                      error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

//------------------------------------------------------------------------------


const
  RS2_API_MAJOR_VERSION = 2;
  RS2_API_MINOR_VERSION = 39;
  RS2_API_PATCH_VERSION = 0;
  RS2_API_BUILD_VERSION = 2337;

  //#ifndef STRINGIFY
  //#define STRINGIFY(arg) #arg
  //#endif
  //#ifndef VAR_ARG_STRING
  //#define VAR_ARG_STRING(arg) STRINGIFY(arg)
  //#endif


(* Versioning rules            : For each release at least one of [MJR/MNR/PTCH] triple is promoted                                            *)
(*                             : Versions that differ by RS2_API_PATCH_VERSION only are interface-compatible, i.e. no user-code changes required *)
(*                             : Versions that differ by MAJOR/MINOR VERSION component can introduce API changes                               *)
(* Version in encoded integer format (1,9,x) -> 01090x. note that each component is limited into [0-99] range by design                        *)
function RS2_API_VERSION(): Cardinal;


(* Return version in "X.Y.Z" format *)
function RS2_API_VERSION_STR(): String;
function RS2_API_FULL_VERSION_STR(): String;

type
(**
 * get the size of rs2_raw_data_buffer
 * \param[in] buffer  pointer to rs2_raw_data_buffer returned by rs2_send_and_receive_raw_data
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return size of rs2_raw_data_buffer
 *)
Trs2_get_raw_data_size = function(const buffer: Prs2_raw_data_buffer; error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(**
 * Delete rs2_raw_data_buffer
 * \param[in] buffer        rs2_raw_data_buffer returned by rs2_send_and_receive_raw_data
 *)
Trs2_delete_raw_data = procedure(const buffer: Prs2_raw_data_buffer); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(**
 * Retrieve char array from rs2_raw_data_buffer
 * \param[in] buffer   rs2_raw_data_buffer returned by rs2_send_and_receive_raw_data
 * \param[out] error   if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return raw data
 *)
Trs2_get_raw_data = function(const buffer: Prs2_raw_data_buffer;
                             error: PPrs2_error): PByte; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(**
 * Retrieve the API version from the source code. Evaluate that the value is conformant to the established policies
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return            the version API encoded into integer value "1.9.3" -> 10903
 *)
Trs2_get_api_version = function(error: PPrs2_error): Integer; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

Trs2_log_to_console = procedure(min_severity: Trs2_log_severity;
                                error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

Trs2_log_to_file = procedure(min_severity: Trs2_log_severity;
                             const file_path: PAnsiChar;
                             error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

Trs2_log_to_callback_cpp = procedure( min_severity: Trs2_log_severity;
                                     callback: Prs2_log_callback;
                                     error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

Trs2_log_to_callback = procedure( min_severity: Trs2_log_severity;
                                  //callback: rs2_log_callback_ptr;
                                  arg: Pointer;
                                  error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};


Trs2_get_log_message_line_number = function( msg: Prs2_log_message;
                                             error: PPrs2_error): LongWord; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};
Trs2_get_log_message_filename = function( msg: Prs2_log_message;
                                          error: PPrs2_error ): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};
Trs2_get_raw_log_message = function( msg: Prs2_log_message;
                                     error: PPrs2_error ): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};
Trs2_get_full_log_message = function( msg: Prs2_log_message;
                                      error: PPrs2_error ): PAnsiChar; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(**
 * Add custom message into librealsense log
 * \param[in] severity  The log level for the message to be written under
 * \param[in] message   Message to be logged
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_log = procedure(severity: Trs2_log_severity;
                     const message: PAnsiChar;
                     error: PPrs2_error); {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(**
 * Given the 2D depth coordinate (x,y) provide the corresponding depth in metric units
 * \param[in] frame_ref  2D depth pixel coordinates (Left-Upper corner origin)
 * \param[in] x,y  2D depth pixel coordinates (Left-Upper corner origin)
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_depth_frame_get_distance = function(const frame_ref: Prs2_frame;
                                         x, y: Integer;
                                         error: PPrs2_error): Single; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(**
 * return the time at specific time point
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return            the time at specific time point, in live and record mode it will return the system time and in playback mode it will return the recorded time
 *)
Trs2_get_time = function(error: PPrs2_error): Trs2_time; {$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

//------------------------------------------------------------------------------
const
  RS2_DEFAULT_TIMEOUT = 15000;

type
(**
 * Create a config instance
 * The config allows pipeline users to request filters for the pipeline streams and device selection and configuration.
 * This is an optional step in pipeline creation, as the pipeline resolves its streaming device internally.
 * Config provides its users a way to set the filters and test if there is no conflict with the pipeline requirements
 * from the device. It also allows the user to find a matching device for the config filters and the pipeline, in order to
 * select a device explicitly, and modify its controls before streaming starts.
 *
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return rs2_config*  A pointer to a new config instance
 *)
Trs2_create_config = function(error: PPrs2_error): Prs2_config;{$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(**
 * Deletes an instance of a config
 *
 * \param[in] config    A pointer to an instance of a config
 *)
Trs2_delete_config = procedure(config: Prs2_config);{$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(**
 * Enable a device stream explicitly, with selected stream parameters.
 * The method allows the application to request a stream with specific configuration. If no stream is explicitly enabled, the pipeline
 * configures the device and its streams according to the attached computer vision modules and processing blocks requirements, or
 * default configuration for the first available device.
 * The application can configure any of the input stream parameters according to its requirement, or set to 0 for don't care value.
 * The config accumulates the application calls for enable configuration methods, until the configuration is applied. Multiple enable
 * stream calls for the same stream with conflicting parameters override each other, and the last call is maintained.
 * Upon calling \c resolve(), the config checks for conflicts between the application configuration requests and the attached computer
 * vision modules and processing blocks requirements, and fails if conflicts are found. Before \c resolve() is called, no conflict
 * check is done.
 *
 * \param[in] config    A pointer to an instance of a config
 * \param[in] stream    Stream type to be enabled
 * \param[in] index     Stream index, used for multiple streams of the same type. -1 indicates any.
 * \param[in] width     Stream image width - for images streams. 0 indicates any.
 * \param[in] height    Stream image height - for images streams. 0 indicates any.
 * \param[in] format    Stream data format - pixel format for images streams, of data type for other streams. RS2_FORMAT_ANY indicates any.
 * \param[in] framerate Stream frames per second. 0 indicates any.
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_config_enable_stream = procedure(config: Prs2_config;
                                      stream: Trs2_stream;
                                      index: Integer;
                                      width: Integer;
                                      height: Integer;
                                      format: Trs2_format;
                                      framerate: Integer;
                                      error: PPrs2_error);{$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(**
 * Enable all device streams explicitly.
 * The conditions and behavior of this method are similar to those of \c enable_stream().
 * This filter enables all raw streams of the selected device. The device is either selected explicitly by the application,
 * or by the pipeline requirements or default. The list of streams is device dependent.
 *
 * \param[in] config    A pointer to an instance of a config
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_config_enable_all_stream = procedure(config: Prs2_config;
                                          error: PPrs2_error);{$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(**
 * Select a specific device explicitly by its serial number, to be used by the pipeline.
 * The conditions and behavior of this method are similar to those of \c enable_stream().
 * This method is required if the application needs to set device or sensor settings prior to pipeline streaming, to enforce
 * the pipeline to use the configured device.
 *
 * \param[in] config    A pointer to an instance of a config
 * \param[in] serial device serial number, as returned by RS2_CAMERA_INFO_SERIAL_NUMBER
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_config_enable_device = procedure(config: Prs2_config;
                                      const serial: PByte;
                                      error: PPrs2_error);{$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(**
 * Select a recorded device from a file, to be used by the pipeline through playback.
 * The device available streams are as recorded to the file, and \c resolve() considers only this device and configuration
 * as available.
 * This request cannot be used if enable_record_to_file() is called for the current config, and vise versa
 * By default, playback is repeated once the file ends. To control this, see 'rs2_config_enable_device_from_file_repeat_option'.
 *
 * \param[in] config    A pointer to an instance of a config
 * \param[in] file      The playback file of the device
 * \param[out] error    if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_config_enable_device_from_file = procedure(config: Prs2_config;
                                                const file_: PAnsiChar;
                                                error: PPrs2_error);{$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(**
 * Select a recorded device from a file, to be used by the pipeline through playback.
 * The device available streams are as recorded to the file, and \c resolve() considers only this device and configuration
 * as available.
 * This request cannot be used if enable_record_to_file() is called for the current config, and vise versa
 *
 * \param[in] config           A pointer to an instance of a config
 * \param[in] file             The playback file of the device
 * \param[in] repeat_playback  if true, when file ends the playback starts again, in an infinite loop;
                               if false, when file ends playback does not start again, and should by stopped manually by the user.
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_config_enable_device_from_file_repeat_option = procedure(config: Prs2_config;
                                                              const file_: PAnsiChar;
                                                              repeat_playback: Integer;
                                                              error: PPrs2_error);{$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(**
 * Requires that the resolved device would be recorded to file
 * This request cannot be used if enable_device_from_file() is called for the current config, and vise versa
 *
 * \param[in] config    A pointer to an instance of a config
 * \param[in] file      The desired file for the output record
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_config_enable_record_to_file = procedure(config: Prs2_config;
                                              const file_: PAnsiChar;
                                              error: PPrs2_error);{$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};


(**
 * Disable a device stream explicitly, to remove any requests on this stream type.
 * The stream can still be enabled due to pipeline computer vision module request. This call removes any filter on the
 * stream configuration.
 *
 * \param[in] config    A pointer to an instance of a config
 * \param[in] stream    Stream type, for which the filters are cleared
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_config_disable_stream = procedure(config: Prs2_config;
                                       stream: Trs2_stream;
                                       error: PPrs2_error);{$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(**
 * Disable a device stream explicitly, to remove any requests on this stream profile.
 * The stream can still be enabled due to pipeline computer vision module request. This call removes any filter on the
 * stream configuration.
 *
 * \param[in] config    A pointer to an instance of a config
 * \param[in] stream    Stream type, for which the filters are cleared
 * \param[in] index     Stream index, for which the filters are cleared
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_config_disable_indexed_stream = procedure(config: Prs2_config;
                                               stream: Trs2_stream;
                                               index: Integer;
                                               error: PPrs2_error);{$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(**
 * Disable all device stream explicitly, to remove any requests on the streams profiles.
 * The streams can still be enabled due to pipeline computer vision module request. This call removes any filter on the
 * streams configuration.
 *
 * \param[in] config    A pointer to an instance of a config
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_config_disable_all_streams = procedure(config: Prs2_config;
                                            error: PPrs2_error);{$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(**
 * Resolve the configuration filters, to find a matching device and streams profiles.
 * The method resolves the user configuration filters for the device and streams, and combines them with the requirements of
 * the computer vision modules and processing blocks attached to the pipeline. If there are no conflicts of requests, it looks
 * for an available device, which can satisfy all requests, and selects the first matching streams configuration. In the absence
 * of any request, the rs2::config selects the first available device and the first color and depth streams configuration.
 * The pipeline profile selection during \c start() follows the same method. Thus, the selected profile is the same, if no
 * change occurs to the available devices occurs.
 * Resolving the pipeline configuration provides the application access to the pipeline selected device for advanced control.
 * The returned configuration is not applied to the device, so the application doesn't own the device sensors. However, the
 * application can call \c enable_device(), to enforce the device returned by this method is selected by pipeline \c start(),
 * and configure the device and sensors options or extensions before streaming starts.
 *
 * \param[in] config    A pointer to an instance of a config
 * \param[in] pipe  The pipeline for which the selected filters are applied
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return       A matching device and streams profile, which satisfies the filters and pipeline requests.
 *)
Trs2_config_resolve = function(config: Prs2_config;
                               pipe: Prs2_pipeline;
                               error: PPrs2_error): Prs2_pipeline_profile;{$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(**
 * Check if the config can resolve the configuration filters, to find a matching device and streams profiles.
 * The resolution conditions are as described in \c resolve().
 *
 * \param[in] config    A pointer to an instance of a config
 * \param[in] pipe  The pipeline for which the selected filters are applied
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return       True if a valid profile selection exists, false if no selection can be found under the config filters and the available devices.
 *)
Trs2_config_can_resolve = function(config: Prs2_config;
                                   pipe: Prs2_pipeline;
                                   error: PPrs2_error): Integer;{$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};
//------------------------------------------------------------------------------

(**
 * Create a pipeline instance
 * The pipeline simplifies the user interaction with the device and computer vision processing modules.
 * The class abstracts the camera configuration and streaming, and the vision modules triggering and threading.
 * It lets the application focus on the computer vision output of the modules, or the device output data.
 * The pipeline can manage computer vision modules, which are implemented as a processing blocks.
 * The pipeline is the consumer of the processing block interface, while the application consumes the
 * computer vision interface.
 * \param[in]  ctx    context
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
Trs2_create_pipeline = function(ctx: Prs2_context;
                                error: PPrs2_error): Prs2_pipeline;{$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(**
 * Stop the pipeline streaming.
 * The pipeline stops delivering samples to the attached computer vision modules and processing blocks, stops the device streaming
 * and releases the device resources used by the pipeline. It is the application's responsibility to release any frame reference it owns.
 * The method takes effect only after \c start() was called, otherwise an exception is raised.
 * \param[in] pipe  pipeline
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 *)
 Trs2_pipeline_stop = procedure(pipe: Prs2_pipeline;
                                error: PPrs2_error);{$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(**
 * Wait until a new set of frames becomes available.
 * The frames set includes time-synchronized frames of each enabled stream in the pipeline.
 * The method blocks the calling thread, and fetches the latest unread frames set.
 * Device frames, which were produced while the function wasn't called, are dropped. To avoid frame drops, this method should be called
 * as fast as the device frame rate.
 * The application can maintain the frames handles to defer processing. However, if the application maintains too long history, the device
 * may lack memory resources to produce new frames, and the following call to this method shall fail to retrieve new frames, until resources
 * are retained.
 * \param[in] pipe the pipeline
 * \param[in] timeout_ms   Max time in milliseconds to wait until an exception will be thrown
 * \param[out] error         if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return Set of coherent frames
 *)
Trs2_pipeline_wait_for_frames = function(pipe: Prs2_pipeline;
                                         timeout_ms: FixedUInt;
                                         error: PPrs2_error): Prs2_frame;{$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(**
 * Check if a new set of frames is available and retrieve the latest undelivered set.
 * The frames set includes time-synchronized frames of each enabled stream in the pipeline.
 * The method returns without blocking the calling thread, with status of new frames available or not. If available, it fetches the
 * latest frames set.
 * Device frames, which were produced while the function wasn't called, are dropped. To avoid frame drops, this method should be called
 * as fast as the device frame rate.
 * The application can maintain the frames handles to defer processing. However, if the application maintains too long history, the device
 * may lack memory resources to produce new frames, and the following calls to this method shall return no new frames, until resources are
 * retained.
 * \param[in] pipe the pipeline
 * \param[out] output_frame frame handle to be released using rs2_release_frame
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return true if new frame was stored to output_frame
 *)
Trs2_pipeline_poll_for_frames = function(pipe: Prs2_pipeline;
                                         output_frame: PPrs2_frame;
                                         error: PPrs2_error): Integer;{$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(**
 * Wait until a new set of frames becomes available.
 * The frames set includes time-synchronized frames of each enabled stream in the pipeline.
 * The method blocks the calling thread, and fetches the latest unread frames set.
 * Device frames, which were produced while the function wasn't called, are dropped. To avoid frame drops, this method should be called
 * as fast as the device frame rate.
 * The application can maintain the frames handles to defer processing. However, if the application maintains too long history, the device
 * may lack memory resources to produce new frames, and the following call to this method shall fail to retrieve new frames, until resources
 * are retained.
 * \param[in] pipe           the pipeline
 * \param[in] timeout_ms     max time in milliseconds to wait until a frame becomes available
 * \param[out] output_frame  frame handle to be released using rs2_release_frame
 * \param[out] error         if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return true if new frame was stored to output_frame
 *)
Trs2_pipeline_try_wait_for_frames = function(pipe: Prs2_pipeline;
                                             output_frame: PPrs2_frame;
                                             timeout_ms: FixedUInt;
                                             error: PPrs2_error): Integer;{$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(**
 * Delete a pipeline instance.
 * Upon destruction, the pipeline will implicitly stop itself
 * \param[in] pipe to delete
 *)
Trs2_delete_pipeline = procedure(pipe: Prs2_pipeline);{$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(**
 * Start the pipeline streaming with its default configuration.
 * The pipeline streaming loop captures samples from the device, and delivers them to the attached computer vision modules
 * and processing blocks, according to each module requirements and threading model.
 * During the loop execution, the application can access the camera streams by calling \c wait_for_frames() or \c poll_for_frames().
 * The streaming loop runs until the pipeline is stopped.
 * Starting the pipeline is possible only when it is not started. If the pipeline was started, an exception is raised.
 *
 * \param[in] pipe    a pointer to an instance of the pipeline
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return             The actual pipeline device and streams profile, which was successfully configured to the streaming device.
 *)
Trs2_pipeline_start = function(pipe: Prs2_pipeline;
                               error: PPrs2_error): Prs2_pipeline_profile;{$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(**
 * Start the pipeline streaming according to the configuraion.
 * The pipeline streaming loop captures samples from the device, and delivers them to the attached computer vision modules
 * and processing blocks, according to each module requirements and threading model.
 * During the loop execution, the application can access the camera streams by calling \c wait_for_frames() or \c poll_for_frames().
 * The streaming loop runs until the pipeline is stopped.
 * Starting the pipeline is possible only when it is not started. If the pipeline was started, an exception is raised.
 * The pipeline selects and activates the device upon start, according to configuration or a default configuration.
 * When the rs2::config is provided to the method, the pipeline tries to activate the config \c resolve() result. If the application
 * requests are conflicting with pipeline computer vision modules or no matching device is available on the platform, the method fails.
 * Available configurations and devices may change between config \c resolve() call and pipeline start, in case devices are connected
 * or disconnected, or another application acquires ownership of a device.
 *
 * \param[in] pipe    a pointer to an instance of the pipeline
 * \param[in] config   A rs2::config with requested filters on the pipeline configuration. By default no filters are applied.
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return             The actual pipeline device and streams profile, which was successfully configured to the streaming device.
 *)
Trs2_pipeline_start_with_config = function(pipe: Prs2_pipeline;
                                           config: Prs2_config;
                                           error: PPrs2_error): Prs2_pipeline_profile;{$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(**
 * Start the pipeline streaming with its default configuration.
 * The pipeline captures samples from the device, and delivers them to the through the provided frame callback.
 * Starting the pipeline is possible only when it is not started. If the pipeline was started, an exception is raised.
 * When starting the pipeline with a callback both \c wait_for_frames() or \c poll_for_frames() will throw exception.
 *
 * \param[in] pipe     A pointer to an instance of the pipeline
 * \param[in] on_frame function pointer to register as per-frame callback
 * \param[in] user auxiliary  data the user wishes to receive together with every frame callback
 * \param[out] error   If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return             The actual pipeline device and streams profile, which was successfully configured to the streaming device.
 *)
Trs2_pipeline_start_with_callback = function(pipe: Prs2_pipeline;
                                             //on_frame: rs2_frame_callback_ptr;
                                             user: Pointer;
                                             error: PPrs2_error): Prs2_pipeline_profile;{$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(**
 * Start the pipeline streaming with its default configuration.
 * The pipeline captures samples from the device, and delivers them to the through the provided frame callback.
 * Starting the pipeline is possible only when it is not started. If the pipeline was started, an exception is raised.
 * When starting the pipeline with a callback both \c wait_for_frames() or \c poll_for_frames() will throw exception.
 *
 * \param[in] pipe     A pointer to an instance of the pipeline
 * \param[in] callback callback object created from c++ application. ownership over the callback object is moved into the relevant streaming lock
 * \param[out] error   If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return             The actual pipeline device and streams profile, which was successfully configured to the streaming device.
 *)
Trs2_pipeline_start_with_callback_cpp = function(pipe: Prs2_pipeline;
                                                 callback: Prs2_frame_callback;
                                                 error: PPrs2_error): Prs2_pipeline_profile;{$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(**
 * Start the pipeline streaming according to the configuraion.
 * The pipeline captures samples from the device, and delivers them to the through the provided frame callback.
 * Starting the pipeline is possible only when it is not started. If the pipeline was started, an exception is raised.
 * When starting the pipeline with a callback both \c wait_for_frames() or \c poll_for_frames() will throw exception.
 * The pipeline selects and activates the device upon start, according to configuration or a default configuration.
 * When the rs2::config is provided to the method, the pipeline tries to activate the config \c resolve() result. If the application
 * requests are conflicting with pipeline computer vision modules or no matching device is available on the platform, the method fails.
 * Available configurations and devices may change between config \c resolve() call and pipeline start, in case devices are connected
 * or disconnected, or another application acquires ownership of a device.
 *
 * \param[in] pipe     A pointer to an instance of the pipeline
 * \param[in] config   A rs2::config with requested filters on the pipeline configuration. By default no filters are applied.
 * \param[in] on_frame function pointer to register as per-frame callback
 * \param[in] user auxiliary  data the user wishes to receive together with every frame callback
 * \param[out] error   If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return             The actual pipeline device and streams profile, which was successfully configured to the streaming device.
 *)
Trs2_pipeline_start_with_config_and_callback = function(pipe: Prs2_pipeline;
                                                        config: Prs2_config;
                                                        //on_frame: rs2_frame_callback_ptr;
                                                        user: Pointer;
                                                        error: PPrs2_error): Prs2_pipeline_profile;{$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(**
 * Start the pipeline streaming according to the configuraion.
 * The pipeline captures samples from the device, and delivers them to the through the provided frame callback.
 * Starting the pipeline is possible only when it is not started. If the pipeline was started, an exception is raised.
 * When starting the pipeline with a callback both \c wait_for_frames() or \c poll_for_frames() will throw exception.
 * The pipeline selects and activates the device upon start, according to configuration or a default configuration.
 * When the rs2::config is provided to the method, the pipeline tries to activate the config \c resolve() result. If the application
 * requests are conflicting with pipeline computer vision modules or no matching device is available on the platform, the method fails.
 * Available configurations and devices may change between config \c resolve() call and pipeline start, in case devices are connected
 * or disconnected, or another application acquires ownership of a device.
 *
 * \param[in] pipe     A pointer to an instance of the pipeline
 * \param[in] config   A rs2::config with requested filters on the pipeline configuration. By default no filters are applied.
 * \param[in] callback callback object created from c++ application. ownership over the callback object is moved into the relevant streaming lock
 * \param[out] error   If non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return             The actual pipeline device and streams profile, which was successfully configured to the streaming device.
 *)
Trs2_pipeline_start_with_config_and_callback_cpp = function(pipe: Prs2_pipeline;
                                                            config: Prs2_config;
                                                            callback: Prs2_frame_callback;
                                                            error: PPrs2_error): Prs2_pipeline_profile;{$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(**
 * Return the active device and streams profiles, used by the pipeline.
 * The pipeline streams profiles are selected during \c start(). The method returns a valid result only when the pipeline is active -
 * between calls to \c start() and \c stop().
 * After \c stop() is called, the pipeline doesn't own the device, thus, the pipeline selected device may change in subsequent activations.
 *
 * \param[in] pipe    a pointer to an instance of the pipeline
 * \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return  The actual pipeline device and streams profile, which was successfully configured to the streaming device on start.
 *)
Trs2_pipeline_get_active_profile = function(pipe: Prs2_pipeline;
                                            error: PPrs2_error): Prs2_pipeline_profile;{$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(**
 * Retrieve the device used by the pipeline.
 * The device class provides the application access to control camera additional settings -
 * get device information, sensor options information, options value query and set, sensor specific extensions.
 * Since the pipeline controls the device streams configuration, activation state and frames reading, calling
 * the device API functions, which execute those operations, results in unexpected behavior.
 * The pipeline streaming device is selected during pipeline \c start(). Devices of profiles, which are not returned by
 * pipeline \c start() or \c get_active_profile(), are not guaranteed to be used by the pipeline.
 *
 * \param[in] profile    A pointer to an instance of a pipeline profile
 * \param[out] error     if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return rs2_device* The pipeline selected device
 *)
Trs2_pipeline_profile_get_device = function(profile: Prs2_pipeline_profile;
                                            error: PPrs2_error): Prs2_device;{$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(**
 * Return the selected streams profiles, which are enabled in this profile.
 *
 * \param[in] profile    A pointer to an instance of a pipeline profile
 * \param[out] error     if non-null, receives any error that occurs during this call, otherwise, errors are ignored
 * \return   list of stream profiles
 *)
Trs2_pipeline_profile_get_streams = function(profile: Prs2_pipeline_profile;
                                             error: PPrs2_error): Prs2_stream_profile_list;{$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};

(**
 * Deletes an instance of a pipeline profile
 *
 * \param[in] profile    A pointer to an instance of a pipeline profile
 *)
Trs2_delete_pipeline_profile = procedure(profile: Prs2_pipeline_profile);{$IFDEF CDECL}cdecl{$ELSE}stdcall{$ENDIF};
//------------------------------------------------------------------------------

var
  rs2_notification_category_to_string: Trs2_notification_category_to_string;
  rs2_exception_type_to_string: Trs2_exception_type_to_string;
  rs2_distortion_to_string: Trs2_distortion_to_string;
  rs2_log_severity_to_string: Trs2_log_severity_to_string;
  rs2_extension_type_to_string: Trs2_extension_type_to_string;
  rs2_extension_to_string: Trs2_extension_to_string;

  rs2_create_error: Trs2_create_error;
  rs2_get_librealsense_exception_type: Trs2_get_librealsense_exception_type;
  rs2_get_failed_function: Trs2_get_failed_function;
  rs2_get_failed_args: Trs2_get_failed_args;
  rs2_get_error_message: Trs2_get_error_message;
  rs2_free_error: Trs2_free_error;

  rs2_create_context: Trs2_create_context;
  rs2_delete_context: Trs2_delete_context;
  rs2_set_devices_changed_callback_cpp: Trs2_set_devices_changed_callback_cpp;
  rs2_set_devices_changed_callback: Trs2_set_devices_changed_callback;
  rs2_context_add_device: Trs2_context_add_device;

  rs2_context_add_software_device: Trs2_context_add_software_device;
  rs2_context_remove_device: Trs2_context_remove_device;
  rs2_query_devices: Trs2_query_devices;
  rs2_query_devices_ex: Trs2_query_devices_ex;
  rs2_create_device_hub: Trs2_create_device_hub;
  rs2_device_hub_wait_for_device: Trs2_device_hub_wait_for_device;
  rs2_device_hub_is_device_connected: Trs2_device_hub_is_device_connected;
  rs2_get_device_count: Trs2_get_device_count;
  rs2_delete_device_list: Trs2_delete_device_list;
  rs2_device_list_contains: Trs2_device_list_contains;
  rs2_create_device: Trs2_create_device;
  rs2_delete_device: Trs2_delete_device;
  rs2_get_device_info: Trs2_get_device_info;
  rs2_supports_device_info: Trs2_supports_device_info;
  rs2_hardware_reset: Trs2_hardware_reset;
  rs2_send_and_receive_raw_data: Trs2_send_and_receive_raw_data;
  rs2_is_device_extendable_to: Trs2_is_device_extendable_to;
  rs2_query_sensors: Trs2_query_sensors;
  rs2_loopback_enable: Trs2_loopback_enable;
  rs2_loopback_disable: Trs2_loopback_disable;
  rs2_loopback_is_enabled: Trs2_loopback_is_enabled;
  rs2_connect_tm2_controller: Trs2_connect_tm2_controller;
  rs2_disconnect_tm2_controller: Trs2_disconnect_tm2_controller;
  rs2_reset_to_factory_calibration: Trs2_reset_to_factory_calibration;
  rs2_write_calibration: Trs2_write_calibration;
  rs2_update_firmware_cpp: Trs2_update_firmware_cpp;
  rs2_update_firmware: Trs2_update_firmware;
  rs2_create_flash_backup_cpp: Trs2_create_flash_backup_cpp;
  rs2_create_flash_backup: Trs2_create_flash_backup;
  rs2_update_firmware_unsigned_cpp: Trs2_update_firmware_unsigned_cpp;
  rs2_update_firmware_unsigned: Trs2_update_firmware_unsigned;
  rs2_enter_update_state: Trs2_enter_update_state;
  rs2_run_on_chip_calibration_cpp: Trs2_run_on_chip_calibration_cpp;
  rs2_run_on_chip_calibration: Trs2_run_on_chip_calibration;
  rs2_run_tare_calibration_cpp: Trs2_run_tare_calibration_cpp;
  rs2_calibration_type_to_string: Trs2_calibration_type_to_string;
  rs2_calibration_status_to_string: Trs2_calibration_status_to_string;
  rs2_register_calibration_change_callback: Trs2_register_calibration_change_callback;
  rs2_register_calibration_change_callback_cpp: Trs2_register_calibration_change_callback_cpp;
  rs2_trigger_device_calibration: Trs2_trigger_device_calibration;
  rs2_run_tare_calibration: Trs2_run_tare_calibration;
  rs2_get_calibration_table: Trs2_get_calibration_table;
  rs2_set_calibration_table: Trs2_set_calibration_table;
  rs2_serialize_json: Trs2_serialize_json;
  rs2_load_json: Trs2_load_json;
  rs2_timestamp_domain_to_string: Trs2_timestamp_domain_to_string;
  rs2_frame_metadata_to_string: Trs2_frame_metadata_to_string;
  rs2_frame_metadata_value_to_string: Trs2_frame_metadata_value_to_string;
  rs2_get_frame_metadata: Trs2_get_frame_metadata;
  rs2_supports_frame_metadata: Trs2_supports_frame_metadata;
  rs2_get_frame_timestamp_domain: Trs2_get_frame_timestamp_domain;
  rs2_get_frame_timestamp: Trs2_get_frame_timestamp;
  rs2_get_frame_sensor: Trs2_get_frame_sensor;
  rs2_get_frame_number: Trs2_get_frame_number;
  rs2_get_frame_data_size: Trs2_get_frame_data_size;
  rs2_get_frame_data: Trs2_get_frame_data;
  rs2_get_frame_width: Trs2_get_frame_width;
  rs2_get_frame_height: Trs2_get_frame_height;
  rs2_depth_frame_get_units: Trs2_depth_frame_get_units;
  rs2_get_frame_stride_in_bytes: Trs2_get_frame_stride_in_bytes;
  rs2_get_frame_bits_per_pixel: Trs2_get_frame_bits_per_pixel;
  rs2_frame_add_ref: Trs2_frame_add_ref;
  rs2_release_frame: Trs2_release_frame;
  rs2_keep_frame: Trs2_keep_frame;
  rs2_get_frame_vertices: Trs2_get_frame_vertices;
  rs2_export_to_ply: Trs2_export_to_ply;
  rs2_get_frame_texture_coordinates: Trs2_get_frame_texture_coordinates;
  rs2_get_frame_points_count: Trs2_get_frame_points_count;
  rs2_get_frame_stream_profile: Trs2_get_frame_stream_profile;
  rs2_is_frame_extendable_to: Trs2_is_frame_extendable_to;
  rs2_allocate_synthetic_video_frame: Trs2_allocate_synthetic_video_frame;
  rs2_allocate_synthetic_motion_frame: Trs2_allocate_synthetic_motion_frame;
  rs2_allocate_points: Trs2_allocate_points;
  rs2_allocate_composite_frame: Trs2_allocate_composite_frame;
  rs2_extract_frame: Trs2_extract_frame;
  rs2_embedded_frames_count: Trs2_embedded_frames_count;
  rs2_synthetic_frame_ready: Trs2_synthetic_frame_ready;
  rs2_pose_frame_get_pose_data: Trs2_pose_frame_get_pose_data;
  rs2_option_to_string: Trs2_option_to_string;
  rs2_sr300_visual_preset_to_string: Trs2_sr300_visual_preset_to_string;
  rs2_rs400_visual_preset_to_string: Trs2_rs400_visual_preset_to_string;
  rs2_l500_visual_preset_to_string: Trs2_l500_visual_preset_to_string;
  rs2_sensor_mode_to_string: Trs2_sensor_mode_to_string;
  rs2_ambient_light_to_string: Trs2_ambient_light_to_string;
  rs2_cah_trigger_to_string: Trs2_cah_trigger_to_string;
  rs2_host_perf_mode_to_string: Trs2_host_perf_mode_to_string;
  rs2_is_option_read_only: Trs2_is_option_read_only;
  rs2_get_option: Trs2_get_option;
  rs2_set_option: Trs2_set_option;
  rs2_get_options_list: Trs2_get_options_list;
  rs2_get_options_list_size: Trs2_get_options_list_size;
  rs2_get_option_name: Trs2_get_option_name;
  rs2_get_option_from_list: Trs2_get_option_from_list;
  rs2_delete_options_list: Trs2_delete_options_list;
  rs2_supports_option: Trs2_supports_option;
  rs2_get_option_range: Trs2_get_option_range;
  rs2_get_option_description: Trs2_get_option_description;
  rs2_get_option_value_description: Trs2_get_option_value_description;

  rs2_create_colorizer: Trs2_create_colorizer;
  rs2_create_sync_processing_block: Trs2_create_sync_processing_block;
  rs2_create_pointcloud: Trs2_create_pointcloud;
  rs2_create_yuy_decoder: Trs2_create_yuy_decoder;
  rs2_create_threshold: Trs2_create_threshold;
  rs2_create_units_transform: Trs2_create_units_transform;
  rs2_create_processing_block: Trs2_create_processing_block;
  rs2_create_processing_block_fptr: Trs2_create_processing_block_fptr;
  rs2_processing_block_register_simple_option: Trs2_processing_block_register_simple_option;
  rs2_start_processing: Trs2_start_processing;
  rs2_start_processing_fptr: Trs2_start_processing_fptr;
  rs2_start_processing_queue: Trs2_start_processing_queue;
  rs2_process_frame: Trs2_process_frame;
  rs2_delete_processing_block: Trs2_delete_processing_block;
  rs2_create_frame_queue: Trs2_create_frame_queue;
  rs2_delete_frame_queue: Trs2_delete_frame_queue;
  rs2_wait_for_frame: Trs2_wait_for_frame;
  rs2_poll_for_frame: Trs2_poll_for_frame;
  rs2_try_wait_for_frame: Trs2_try_wait_for_frame;
  rs2_enqueue_frame: Trs2_enqueue_frame;
  rs2_create_align: Trs2_create_align;
  rs2_create_decimation_filter_block: Trs2_create_decimation_filter_block;
  rs2_create_temporal_filter_block: Trs2_create_temporal_filter_block;
  rs2_create_spatial_filter_block: Trs2_create_spatial_filter_block;
  rs2_create_disparity_transform_block: Trs2_create_disparity_transform_block;
  rs2_create_hole_filling_filter_block: Trs2_create_hole_filling_filter_block;
  rs2_create_rates_printer_block: Trs2_create_rates_printer_block;
  rs2_create_zero_order_invalidation_block: Trs2_create_zero_order_invalidation_block;
  rs2_create_huffman_depth_decompress_block: Trs2_create_huffman_depth_decompress_block;
  rs2_create_hdr_merge_processing_block: Trs2_create_hdr_merge_processing_block;
  rs2_create_sequence_id_filter: Trs2_create_sequence_id_filter;
  rs2_get_processing_block_info: Trs2_get_processing_block_info;
  rs2_supports_processing_block_info: Trs2_supports_processing_block_info;
  rs2_is_processing_block_extendable_to: Trs2_is_processing_block_extendable_to;

  rs2_playback_status_to_string: Trs2_playback_status_to_string;
  rs2_create_record_device: Trs2_create_record_device;
  rs2_create_record_device_ex: Trs2_create_record_device_ex;
  rs2_record_device_pause: Trs2_record_device_pause;
  rs2_record_device_resume: Trs2_record_device_resume;
  rs2_record_device_filename: Trs2_record_device_filename;
  rs2_create_playback_device: Trs2_create_playback_device;
  rs2_playback_device_get_file_path: Trs2_playback_device_get_file_path;
  rs2_playback_get_duration: Trs2_playback_get_duration;
  rs2_playback_seek: Trs2_playback_seek;
  rs2_playback_get_position: Trs2_playback_get_position;
  rs2_playback_device_resume: Trs2_playback_device_resume;
  rs2_playback_device_pause: Trs2_playback_device_pause;
  rs2_playback_device_set_real_time: Trs2_playback_device_set_real_time;
  rs2_playback_device_is_real_time: Trs2_playback_device_is_real_time;
  rs2_playback_device_set_status_changed_callback: Trs2_playback_device_set_status_changed_callback;
  rs2_playback_device_get_current_status: Trs2_playback_device_get_current_status;
  rs2_playback_device_set_playback_speed: Trs2_playback_device_set_playback_speed;
  rs2_playback_device_stop: Trs2_playback_device_stop;


  rs2_camera_info_to_string: Trs2_camera_info_to_string;
  rs2_stream_to_string: Trs2_stream_to_string;
  rs2_format_to_string: Trs2_format_to_string;
  rs2_delete_sensor_list: Trs2_delete_sensor_list;
  rs2_get_sensors_count: Trs2_get_sensors_count;
  rs2_delete_sensor: Trs2_delete_sensor;
  rs2_create_sensor: Trs2_create_sensor;
  rs2_create_device_from_sensor: Trs2_create_device_from_sensor;
  rs2_get_sensor_info: Trs2_get_sensor_info;
  rs2_supports_sensor_info: Trs2_supports_sensor_info;
  rs2_is_sensor_extendable_to: Trs2_is_sensor_extendable_to;
  rs2_get_depth_scale: Trs2_get_depth_scale;
  rs2_depth_stereo_frame_get_baseline: Trs2_depth_stereo_frame_get_baseline;
  rs2_get_stereo_baseline: Trs2_get_stereo_baseline;
  rs2_set_region_of_interest: Trs2_set_region_of_interest;
  rs2_get_region_of_interest: Trs2_get_region_of_interest;
  rs2_open: Trs2_open;
  rs2_open_multiple: Trs2_open_multiple;
  rs2_close: Trs2_close;
  rs2_start: Trs2_start;
  rs2_start_cpp: Trs2_start_cpp;
  rs2_start_queue: Trs2_start_queue;
  rs2_stop: Trs2_stop;
  rs2_set_notifications_callback: Trs2_set_notifications_callback;
  rs2_set_notifications_callback_cpp: Trs2_set_notifications_callback_cpp;
  rs2_get_notification_description: Trs2_get_notification_description;
  rs2_get_notification_timestamp: Trs2_get_notification_timestamp;
  rs2_get_notification_severity: Trs2_get_notification_severity;
  rs2_get_notification_category: Trs2_get_notification_category;
  rs2_get_notification_serialized_data: Trs2_get_notification_serialized_data;
  rs2_get_stream_profiles: Trs2_get_stream_profiles;
  rs2_get_active_streams: Trs2_get_active_streams;
  rs2_get_stream_profile: Trs2_get_stream_profile;
  rs2_get_stream_profile_data: Trs2_get_stream_profile_data;
  rs2_set_stream_profile_data: Trs2_set_stream_profile_data;
  rs2_clone_stream_profile: Trs2_clone_stream_profile;
  rs2_clone_video_stream_profile: Trs2_clone_video_stream_profile;
  rs2_delete_stream_profile: Trs2_delete_stream_profile;
  rs2_stream_profile_is: Trs2_stream_profile_is;
  rs2_get_video_stream_resolution: Trs2_get_video_stream_resolution;
  rs2_get_motion_intrinsics: Trs2_get_motion_intrinsics;
  rs2_is_stream_profile_default: Trs2_is_stream_profile_default;
  rs2_get_stream_profiles_count: Trs2_get_stream_profiles_count;
  rs2_delete_stream_profiles_list: Trs2_delete_stream_profiles_list;
  rs2_get_extrinsics: Trs2_get_extrinsics;
  rs2_register_extrinsics: Trs2_register_extrinsics;
  rs2_override_extrinsics: Trs2_override_extrinsics;
  rs2_get_video_stream_intrinsics: Trs2_get_video_stream_intrinsics;
  rs2_get_recommended_processing_blocks: Trs2_get_recommended_processing_blocks;
  rs2_get_processing_block: Trs2_get_processing_block;
  rs2_get_recommended_processing_blocks_count: Trs2_get_recommended_processing_blocks_count;
  rs2_delete_recommended_processing_blocks: Trs2_delete_recommended_processing_blocks;
  rs2_import_localization_map: Trs2_import_localization_map;
  rs2_export_localization_map: Trs2_export_localization_map;
  rs2_set_static_node: Trs2_set_static_node;
  rs2_get_static_node: Trs2_get_static_node;
  rs2_remove_static_node: Trs2_remove_static_node;
  rs2_load_wheel_odometry_config: Trs2_load_wheel_odometry_config;
  rs2_send_wheel_odometry: Trs2_send_wheel_odometry;
  rs2_set_intrinsics: Trs2_set_intrinsics;
  rs2_override_intrinsics: Trs2_override_intrinsics;
  rs2_set_extrinsics: Trs2_set_extrinsics;
  rs2_get_dsm_params: Trs2_get_dsm_params;
  rs2_override_dsm_params: Trs2_override_dsm_params;
  rs2_reset_sensor_calibration: Trs2_reset_sensor_calibration;
  rs2_set_motion_device_intrinsics: Trs2_set_motion_device_intrinsics;
  rs2_get_raw_data_size: Trs2_get_raw_data_size;
  rs2_delete_raw_data: Trs2_delete_raw_data;
  rs2_get_raw_data: Trs2_get_raw_data;
  rs2_get_api_version: Trs2_get_api_version;
  rs2_log_to_console: Trs2_log_to_console;
  rs2_log_to_file: Trs2_log_to_file;
  rs2_log_to_callback_cpp: Trs2_log_to_callback_cpp;
  rs2_log_to_callback: Trs2_log_to_callback;
  rs2_get_log_message_line_number: Trs2_get_log_message_line_number;
  rs2_get_log_message_filename: Trs2_get_log_message_filename;
  rs2_get_raw_log_message: Trs2_get_raw_log_message;
  rs2_get_full_log_message: Trs2_get_full_log_message;
  rs2_log: Trs2_log;
  rs2_depth_frame_get_distance: Trs2_depth_frame_get_distance;
  rs2_get_time: Trs2_get_time;

  rs2_create_config: Trs2_create_config;
  rs2_delete_config: Trs2_delete_config;
  rs2_config_enable_stream: Trs2_config_enable_stream;
  rs2_config_enable_all_stream: Trs2_config_enable_all_stream;
  rs2_config_enable_device: Trs2_config_enable_device;
  rs2_config_enable_device_from_file: Trs2_config_enable_device_from_file;
  rs2_config_enable_device_from_file_repeat_option: Trs2_config_enable_device_from_file_repeat_option;
  rs2_config_enable_record_to_file: Trs2_config_enable_record_to_file;
  rs2_config_disable_stream: Trs2_config_disable_stream;
  rs2_config_disable_indexed_stream: Trs2_config_disable_indexed_stream;
  rs2_config_disable_all_streams: Trs2_config_disable_all_streams;
  rs2_config_resolve: Trs2_config_resolve;
  rs2_config_can_resolve: Trs2_config_can_resolve;

  rs2_create_pipeline: Trs2_create_pipeline;
  rs2_pipeline_stop: Trs2_pipeline_stop;
  rs2_pipeline_wait_for_frames: Trs2_pipeline_wait_for_frames;
  rs2_pipeline_poll_for_frames: Trs2_pipeline_poll_for_frames;
  rs2_pipeline_try_wait_for_frames: Trs2_pipeline_try_wait_for_frames;
  rs2_delete_pipeline: Trs2_delete_pipeline;
  rs2_pipeline_start: Trs2_pipeline_start;
  rs2_pipeline_start_with_config: Trs2_pipeline_start_with_config;
  rs2_pipeline_start_with_callback: Trs2_pipeline_start_with_callback;
  rs2_pipeline_start_with_callback_cpp: Trs2_pipeline_start_with_callback_cpp;
  rs2_pipeline_start_with_config_and_callback: Trs2_pipeline_start_with_config_and_callback;
  rs2_pipeline_start_with_config_and_callback_cpp: Trs2_pipeline_start_with_config_and_callback_cpp;
  rs2_pipeline_get_active_profile: Trs2_pipeline_get_active_profile;
  rs2_pipeline_profile_get_device: Trs2_pipeline_profile_get_device;
  rs2_pipeline_profile_get_streams: Trs2_pipeline_profile_get_streams;
  rs2_delete_pipeline_profile: Trs2_delete_pipeline_profile;


function rs2_GetProcAddress(ProcName: PAnsiChar; LibHandle: Pointer = nil): Pointer;
function InitLibRealsense2(LibName: String = LibRealsense2_Name): Boolean;

var
  RS2_LibHandle: Pointer = nil;

implementation

function RS2_API_VERSION(): Cardinal;
begin
  // #define RS2_API_VERSION  (((RS2_API_MAJOR_VERSION) * 10000) + ((RS2_API_MINOR_VERSION) * 100) + (RS2_API_PATCH_VERSION))
  Result := RS2_API_MAJOR_VERSION * 10000 + RS2_API_MINOR_VERSION * 100 + RS2_API_PATCH_VERSION;
end;

function RS2_API_VERSION_STR(): String;
begin
  //#define RS2_API_VERSION_STR (VAR_ARG_STRING(RS2_API_MAJOR_VERSION.RS2_API_MINOR_VERSION.RS2_API_PATCH_VERSION))
  Result := IntToStr(RS2_API_MAJOR_VERSION) + '.' +
  IntToStr(RS2_API_MINOR_VERSION) + '.' + IntToStr(RS2_API_PATCH_VERSION);
end;

function RS2_API_FULL_VERSION_STR(): String;
begin
  //#define RS2_API_FULL_VERSION_STR (VAR_ARG_STRING(RS2_API_MAJOR_VERSION.RS2_API_MINOR_VERSION.RS2_API_PATCH_VERSION.RS2_API_BUILD_VERSION))
  Result := IntToStr(RS2_API_MAJOR_VERSION) + '.' +
  IntToStr(RS2_API_MINOR_VERSION) + '.' + IntToStr(RS2_API_PATCH_VERSION) + '.' +
  IntToStr(RS2_API_BUILD_VERSION);
end;

{$IFDEF LINUX}
const
  RTLD_LAZY = $001;
  RTLD_NOW = $002;
  RTLD_BINDING_MASK = $003;

  // Seems to work on Debian / Fedora
  LibraryLib = {$IFDEF LINUX} 'libdl.so.2'{$ELSE} 'c'{$ENDIF};

function dlopen(Name: PAnsiChar; Flags: LongInt): Pointer; cdecl; external LibraryLib name 'dlopen';
function dlclose(Lib: Pointer): LongInt; cdecl; external LibraryLib name 'dlclose';
function dlsym(Lib: Pointer; Name: PAnsiChar): Pointer; cdecl; external LibraryLib name 'dlsym';
{$ENDIF}

function rs2_LoadLibrary(Name: PChar): Pointer;
begin
  {$IFDEF WINDOWS}
    Result := Pointer(LoadLibrary(Name));
  {$ENDIF}

  {$IFDEF LINUX}
    Result := dlopen(Name, RTLD_LAZY);
  {$ENDIF}
end;

function rs2_FreeLibrary(LibHandle: Pointer): Boolean;
begin
  if LibHandle = nil then
    Result := False
  else
  {$IFDEF WINDOWS}
    Result := FreeLibrary(HMODULE(LibHandle));
  {$ENDIF}

  {$IFDEF LINUX}
    Result := dlclose(LibHandle) = 0;
  {$ENDIF}
end;

function rs2_GetProcAddress(ProcName: PAnsiChar; LibHandle: Pointer = nil): Pointer;
begin
  if LibHandle = nil then LibHandle := RS2_LibHandle;

  {$IFDEF WINDOWS}
    Result := GetProcAddress(HMODULE(LibHandle), ProcName);
  {$ENDIF}

  {$IFDEF LINUX}
    Result := dlsym(LibHandle, ProcName);
  {$ENDIF}
end;

function InitLibRealsense2(LibName: String): Boolean;
begin
  Result := False;

  // free opened libraries
  if RS2_LibHandle <> nil then
    rs2_FreeLibrary(RS2_LibHandle);

  // load library
  RS2_LibHandle := rs2_LoadLibrary(PChar(LibName));

  // load librealsense2 functions
  if (RS2_LibHandle <> nil) then
  begin
    rs2_notification_category_to_string := Trs2_notification_category_to_string(rs2_GetProcAddress('rs2_notification_category_to_string', RS2_LibHandle));
    rs2_exception_type_to_string :=  Trs2_exception_type_to_string(rs2_GetProcAddress('rs2_exception_type_to_string', RS2_LibHandle));
    rs2_distortion_to_string := Trs2_distortion_to_string(rs2_GetProcAddress('rs2_distortion_to_string', RS2_LibHandle));
    rs2_log_severity_to_string := Trs2_log_severity_to_string(rs2_GetProcAddress('rs2_log_severity_to_string', RS2_LibHandle));
    rs2_extension_type_to_string := Trs2_extension_type_to_string(rs2_GetProcAddress('rs2_extension_type_to_string', RS2_LibHandle));
    rs2_extension_to_string := Trs2_extension_to_string(rs2_GetProcAddress('rs2_extension_to_string', RS2_LibHandle));

    rs2_create_error := Trs2_create_error(rs2_GetProcAddress('rs2_create_error', RS2_LibHandle));
    rs2_get_librealsense_exception_type := Trs2_get_librealsense_exception_type(rs2_GetProcAddress('rs2_get_librealsense_exception_type', RS2_LibHandle));
    rs2_get_failed_function := Trs2_get_failed_function(rs2_GetProcAddress('rs2_get_failed_function', RS2_LibHandle));
    rs2_get_failed_args := Trs2_get_failed_args(rs2_GetProcAddress('rs2_get_failed_args', RS2_LibHandle));
    rs2_get_error_message := Trs2_get_error_message(rs2_GetProcAddress('rs2_get_error_message', RS2_LibHandle));
    rs2_free_error := Trs2_free_error(rs2_GetProcAddress('rs2_free_error', RS2_LibHandle));

    rs2_create_context := Trs2_create_context(rs2_GetProcAddress('rs2_create_context', RS2_LibHandle));
    rs2_delete_context := Trs2_delete_context(rs2_GetProcAddress('rs2_delete_context', RS2_LibHandle));
    rs2_set_devices_changed_callback_cpp := Trs2_set_devices_changed_callback_cpp(rs2_GetProcAddress('rs2_set_devices_changed_callback_cpp', RS2_LibHandle));
    rs2_set_devices_changed_callback := Trs2_set_devices_changed_callback(rs2_GetProcAddress('rs2_set_devices_changed_callback', RS2_LibHandle));
    rs2_context_add_device := Trs2_context_add_device(rs2_GetProcAddress('rs2_context_add_device', RS2_LibHandle));

    rs2_context_add_software_device := Trs2_context_add_software_device(rs2_GetProcAddress('rs2_context_add_software_device', RS2_LibHandle));
    rs2_context_remove_device := Trs2_context_remove_device(rs2_GetProcAddress('rs2_context_remove_device', RS2_LibHandle));
    rs2_query_devices := Trs2_query_devices(rs2_GetProcAddress('rs2_query_devices', RS2_LibHandle));
    rs2_query_devices_ex := Trs2_query_devices_ex(rs2_GetProcAddress('rs2_query_devices_ex', RS2_LibHandle));
    rs2_create_device_hub := Trs2_create_device_hub(rs2_GetProcAddress('rs2_create_device_hub', RS2_LibHandle));
    rs2_device_hub_wait_for_device := Trs2_device_hub_wait_for_device(rs2_GetProcAddress('rs2_device_hub_wait_for_device', RS2_LibHandle));
    rs2_device_hub_is_device_connected := Trs2_device_hub_is_device_connected(rs2_GetProcAddress('rs2_device_hub_is_device_connected', RS2_LibHandle));
    rs2_get_device_count := Trs2_get_device_count(rs2_GetProcAddress('rs2_get_device_count', RS2_LibHandle));
    rs2_delete_device_list := Trs2_delete_device_list(rs2_GetProcAddress('rs2_delete_device_list', RS2_LibHandle));
    rs2_device_list_contains := Trs2_device_list_contains(rs2_GetProcAddress('rs2_device_list_contains', RS2_LibHandle));
    rs2_create_device := Trs2_create_device(rs2_GetProcAddress('rs2_create_device', RS2_LibHandle));
    rs2_delete_device := Trs2_delete_device(rs2_GetProcAddress('rs2_delete_device', RS2_LibHandle));
    rs2_get_device_info := Trs2_get_device_info(rs2_GetProcAddress('rs2_get_device_info', RS2_LibHandle));
    rs2_supports_device_info := Trs2_supports_device_info(rs2_GetProcAddress('rs2_supports_device_info', RS2_LibHandle));
    rs2_hardware_reset := Trs2_hardware_reset(rs2_GetProcAddress('rs2_send_and_receive_raw_data', RS2_LibHandle));
    rs2_send_and_receive_raw_data := Trs2_send_and_receive_raw_data(rs2_GetProcAddress('rs2_send_and_receive_raw_data', RS2_LibHandle));
    rs2_is_device_extendable_to := Trs2_is_device_extendable_to(rs2_GetProcAddress('rs2_is_device_extendable_to', RS2_LibHandle));
    rs2_query_sensors := Trs2_query_sensors(rs2_GetProcAddress('rs2_query_sensors', RS2_LibHandle));
    rs2_loopback_enable := Trs2_loopback_enable(rs2_GetProcAddress('rs2_loopback_enable', RS2_LibHandle));
    rs2_loopback_disable := Trs2_loopback_disable(rs2_GetProcAddress('rs2_loopback_disable', RS2_LibHandle));
    rs2_loopback_is_enabled := Trs2_loopback_is_enabled(rs2_GetProcAddress('rs2_loopback_is_enabled', RS2_LibHandle));
    rs2_connect_tm2_controller := Trs2_connect_tm2_controller(rs2_GetProcAddress('rs2_connect_tm2_controller', RS2_LibHandle));
    rs2_disconnect_tm2_controller := Trs2_disconnect_tm2_controller(rs2_GetProcAddress('rs2_disconnect_tm2_controller', RS2_LibHandle));
    rs2_reset_to_factory_calibration := Trs2_reset_to_factory_calibration(rs2_GetProcAddress('rs2_reset_to_factory_calibration', RS2_LibHandle));
    rs2_write_calibration := Trs2_write_calibration(rs2_GetProcAddress('rs2_write_calibration', RS2_LibHandle));
    rs2_update_firmware_cpp := Trs2_update_firmware_cpp(rs2_GetProcAddress('rs2_update_firmware_cpp', RS2_LibHandle));
    rs2_update_firmware := Trs2_update_firmware(rs2_GetProcAddress('rs2_update_firmware', RS2_LibHandle));
    rs2_create_flash_backup_cpp := Trs2_create_flash_backup_cpp(rs2_GetProcAddress('rs2_create_flash_backup_cpp', RS2_LibHandle));
    rs2_create_flash_backup := Trs2_create_flash_backup(rs2_GetProcAddress('rs2_create_flash_backup', RS2_LibHandle));
    rs2_update_firmware_unsigned_cpp := Trs2_update_firmware_unsigned_cpp(rs2_GetProcAddress('rs2_update_firmware_unsigned_cpp', RS2_LibHandle));
    rs2_update_firmware_unsigned := Trs2_update_firmware_unsigned(rs2_GetProcAddress('rs2_update_firmware_unsigned', RS2_LibHandle));
    rs2_enter_update_state := Trs2_enter_update_state(rs2_GetProcAddress('rs2_enter_update_state', RS2_LibHandle));
    rs2_run_on_chip_calibration_cpp := Trs2_run_on_chip_calibration_cpp(rs2_GetProcAddress('rs2_run_on_chip_calibration_cpp', RS2_LibHandle));
    rs2_run_on_chip_calibration := Trs2_run_on_chip_calibration(rs2_GetProcAddress('rs2_run_on_chip_calibration', RS2_LibHandle));
    rs2_run_tare_calibration_cpp := Trs2_run_tare_calibration_cpp(rs2_GetProcAddress('rs2_run_tare_calibration_cpp', RS2_LibHandle));
    rs2_calibration_type_to_string := Trs2_calibration_type_to_string(rs2_GetProcAddress('rs2_calibration_type_to_string', RS2_LibHandle));
    rs2_calibration_status_to_string := Trs2_calibration_status_to_string(rs2_GetProcAddress('rs2_calibration_status_to_string', RS2_LibHandle));
    rs2_register_calibration_change_callback := Trs2_register_calibration_change_callback(rs2_GetProcAddress('rs2_register_calibration_change_callback', RS2_LibHandle));
    rs2_register_calibration_change_callback_cpp := Trs2_register_calibration_change_callback_cpp(rs2_GetProcAddress('rs2_register_calibration_change_callback_cpp', RS2_LibHandle));
    rs2_trigger_device_calibration := Trs2_trigger_device_calibration(rs2_GetProcAddress('rs2_trigger_device_calibration', RS2_LibHandle));
    rs2_run_tare_calibration := Trs2_run_tare_calibration(rs2_GetProcAddress('rs2_run_tare_calibration', RS2_LibHandle));
    rs2_get_calibration_table := Trs2_get_calibration_table(rs2_GetProcAddress('rs2_get_calibration_table', RS2_LibHandle));
    rs2_set_calibration_table := Trs2_set_calibration_table(rs2_GetProcAddress('rs2_set_calibration_table', RS2_LibHandle));
    rs2_serialize_json := Trs2_serialize_json(rs2_GetProcAddress('rs2_serialize_json', RS2_LibHandle));
    rs2_load_json := Trs2_load_json(rs2_GetProcAddress('rs2_load_json', RS2_LibHandle));
    rs2_timestamp_domain_to_string := Trs2_timestamp_domain_to_string(rs2_GetProcAddress('rs2_timestamp_domain_to_string', RS2_LibHandle));
    rs2_frame_metadata_to_string := Trs2_frame_metadata_to_string(rs2_GetProcAddress('rs2_frame_metadata_to_string', RS2_LibHandle));
    rs2_frame_metadata_value_to_string := Trs2_frame_metadata_value_to_string(rs2_GetProcAddress('rs2_frame_metadata_value_to_string', RS2_LibHandle));
    rs2_get_frame_metadata := Trs2_get_frame_metadata(rs2_GetProcAddress('rs2_get_frame_metadata', RS2_LibHandle));
    rs2_supports_frame_metadata := Trs2_supports_frame_metadata(rs2_GetProcAddress('rs2_supports_frame_metadata', RS2_LibHandle));
    rs2_get_frame_timestamp_domain := Trs2_get_frame_timestamp_domain(rs2_GetProcAddress('rs2_get_frame_timestamp_domain', RS2_LibHandle));
    rs2_get_frame_timestamp := Trs2_get_frame_timestamp(rs2_GetProcAddress('rs2_get_frame_timestamp', RS2_LibHandle));
    rs2_get_frame_sensor := Trs2_get_frame_sensor(rs2_GetProcAddress('rs2_get_frame_sensor', RS2_LibHandle));
    rs2_get_frame_number := Trs2_get_frame_number(rs2_GetProcAddress('rs2_get_frame_number', RS2_LibHandle));
    rs2_get_frame_data_size := Trs2_get_frame_data_size(rs2_GetProcAddress('rs2_get_frame_data_size', RS2_LibHandle));
    rs2_get_frame_data := Trs2_get_frame_data(rs2_GetProcAddress('rs2_get_frame_data', RS2_LibHandle));
    rs2_get_frame_width := Trs2_get_frame_width(rs2_GetProcAddress('rs2_get_frame_width', RS2_LibHandle));
    rs2_get_frame_height := Trs2_get_frame_height(rs2_GetProcAddress('rs2_get_frame_height', RS2_LibHandle));
    rs2_depth_frame_get_units := Trs2_depth_frame_get_units(rs2_GetProcAddress('rs2_depth_frame_get_units', RS2_LibHandle));
    rs2_get_frame_stride_in_bytes := Trs2_get_frame_stride_in_bytes(rs2_GetProcAddress('rs2_get_frame_stride_in_bytes', RS2_LibHandle));
    rs2_get_frame_bits_per_pixel := Trs2_get_frame_bits_per_pixel(rs2_GetProcAddress('rs2_get_frame_bits_per_pixel', RS2_LibHandle));
    rs2_frame_add_ref := Trs2_frame_add_ref(rs2_GetProcAddress('rs2_frame_add_ref', RS2_LibHandle));
    rs2_release_frame := Trs2_release_frame(rs2_GetProcAddress('rs2_release_frame', RS2_LibHandle));
    rs2_keep_frame := Trs2_keep_frame(rs2_GetProcAddress('rs2_keep_frame', RS2_LibHandle));
    rs2_get_frame_vertices := Trs2_get_frame_vertices(rs2_GetProcAddress('rs2_get_frame_vertices', RS2_LibHandle));
    rs2_export_to_ply := Trs2_export_to_ply(rs2_GetProcAddress('rs2_export_to_ply', RS2_LibHandle));
    rs2_get_frame_texture_coordinates := Trs2_get_frame_texture_coordinates(rs2_GetProcAddress('rs2_get_frame_texture_coordinates', RS2_LibHandle));
    rs2_get_frame_points_count := Trs2_get_frame_points_count(rs2_GetProcAddress('rs2_get_frame_points_count', RS2_LibHandle));
    rs2_get_frame_stream_profile := Trs2_get_frame_stream_profile(rs2_GetProcAddress('rs2_get_frame_stream_profile', RS2_LibHandle));
    rs2_is_frame_extendable_to := Trs2_is_frame_extendable_to(rs2_GetProcAddress('rs2_is_frame_extendable_to', RS2_LibHandle));
    rs2_allocate_synthetic_video_frame := Trs2_allocate_synthetic_video_frame(rs2_GetProcAddress('rs2_allocate_synthetic_video_frame', RS2_LibHandle));
    rs2_allocate_synthetic_motion_frame := Trs2_allocate_synthetic_motion_frame(rs2_GetProcAddress('rs2_allocate_synthetic_motion_frame', RS2_LibHandle));
    rs2_allocate_points := Trs2_allocate_points(rs2_GetProcAddress('rs2_allocate_points', RS2_LibHandle));
    rs2_allocate_composite_frame := Trs2_allocate_composite_frame(rs2_GetProcAddress('rs2_allocate_composite_frame', RS2_LibHandle));
    rs2_extract_frame := Trs2_extract_frame(rs2_GetProcAddress('rs2_extract_frame', RS2_LibHandle));
    rs2_embedded_frames_count := Trs2_embedded_frames_count(rs2_GetProcAddress('rs2_embedded_frames_count', RS2_LibHandle));
    rs2_synthetic_frame_ready := Trs2_synthetic_frame_ready(rs2_GetProcAddress('rs2_synthetic_frame_ready', RS2_LibHandle));
    rs2_pose_frame_get_pose_data := Trs2_pose_frame_get_pose_data(rs2_GetProcAddress('rs2_pose_frame_get_pose_data', RS2_LibHandle));
    rs2_option_to_string := Trs2_option_to_string(rs2_GetProcAddress('rs2_option_to_string', RS2_LibHandle));
    rs2_sr300_visual_preset_to_string := Trs2_sr300_visual_preset_to_string(rs2_GetProcAddress('rs2_sr300_visual_preset_to_string', RS2_LibHandle));
    rs2_rs400_visual_preset_to_string := Trs2_rs400_visual_preset_to_string(rs2_GetProcAddress('rs2_rs400_visual_preset_to_string', RS2_LibHandle));
    rs2_l500_visual_preset_to_string := Trs2_l500_visual_preset_to_string(rs2_GetProcAddress('rs2_l500_visual_preset_to_string', RS2_LibHandle));
    rs2_sensor_mode_to_string := Trs2_sensor_mode_to_string(rs2_GetProcAddress('rs2_sensor_mode_to_string', RS2_LibHandle));
    rs2_ambient_light_to_string := Trs2_ambient_light_to_string(rs2_GetProcAddress('rs2_ambient_light_to_string', RS2_LibHandle));
    rs2_cah_trigger_to_string := Trs2_cah_trigger_to_string(rs2_GetProcAddress('rs2_cah_trigger_to_string', RS2_LibHandle));
    rs2_host_perf_mode_to_string := Trs2_host_perf_mode_to_string(rs2_GetProcAddress('rs2_host_perf_mode_to_string', RS2_LibHandle));
    rs2_is_option_read_only := Trs2_is_option_read_only(rs2_GetProcAddress('rs2_is_option_read_only', RS2_LibHandle));
    rs2_get_option := Trs2_get_option(rs2_GetProcAddress('rs2_get_option', RS2_LibHandle));
    rs2_set_option := Trs2_set_option(rs2_GetProcAddress('rs2_set_option', RS2_LibHandle));
    rs2_get_options_list := Trs2_get_options_list(rs2_GetProcAddress('rs2_get_options_list', RS2_LibHandle));
    rs2_get_options_list_size := Trs2_get_options_list_size(rs2_GetProcAddress('rs2_get_options_list_size', RS2_LibHandle));
    rs2_get_option_name := Trs2_get_option_name(rs2_GetProcAddress('rs2_get_option_name', RS2_LibHandle));
    rs2_get_option_from_list := Trs2_get_option_from_list(rs2_GetProcAddress('rs2_get_option_from_list', RS2_LibHandle));
    rs2_delete_options_list := Trs2_delete_options_list(rs2_GetProcAddress('rs2_delete_options_list', RS2_LibHandle));
    rs2_supports_option := Trs2_supports_option(rs2_GetProcAddress('rs2_supports_option', RS2_LibHandle));
    rs2_get_option_range := Trs2_get_option_range(rs2_GetProcAddress('rs2_get_option_range', RS2_LibHandle));
    rs2_get_option_description := Trs2_get_option_description(rs2_GetProcAddress('rs2_get_option_description', RS2_LibHandle));
    rs2_get_option_value_description := Trs2_get_option_value_description(rs2_GetProcAddress('rs2_get_option_value_description', RS2_LibHandle));

    rs2_create_colorizer := Trs2_create_colorizer(rs2_GetProcAddress('rs2_create_colorizer', RS2_LibHandle));
    rs2_create_sync_processing_block := Trs2_create_sync_processing_block(rs2_GetProcAddress('rs2_create_sync_processing_block', RS2_LibHandle));
    rs2_create_pointcloud := Trs2_create_pointcloud(rs2_GetProcAddress('rs2_create_pointcloud', RS2_LibHandle));
    rs2_create_yuy_decoder := Trs2_create_yuy_decoder(rs2_GetProcAddress('rs2_create_yuy_decoder', RS2_LibHandle));
    rs2_create_threshold := Trs2_create_threshold(rs2_GetProcAddress('rs2_create_threshold', RS2_LibHandle));
    rs2_create_units_transform := Trs2_create_units_transform(rs2_GetProcAddress('rs2_create_units_transform', RS2_LibHandle));
    rs2_create_processing_block := Trs2_create_processing_block(rs2_GetProcAddress('rs2_create_processing_block', RS2_LibHandle));
    rs2_create_processing_block_fptr := Trs2_create_processing_block_fptr(rs2_GetProcAddress('rs2_create_processing_block_fptr', RS2_LibHandle));
    rs2_processing_block_register_simple_option := Trs2_processing_block_register_simple_option(rs2_GetProcAddress('rs2_processing_block_register_simple_option', RS2_LibHandle));
    rs2_start_processing := Trs2_start_processing(rs2_GetProcAddress('rs2_start_processing', RS2_LibHandle));
    rs2_start_processing_fptr := Trs2_start_processing_fptr(rs2_GetProcAddress('rs2_start_processing_fptr', RS2_LibHandle));
    rs2_start_processing_queue := Trs2_start_processing_queue(rs2_GetProcAddress('rs2_start_processing_queue', RS2_LibHandle));
    rs2_process_frame := Trs2_process_frame(rs2_GetProcAddress('rs2_process_frame', RS2_LibHandle));
    rs2_delete_processing_block := Trs2_delete_processing_block(rs2_GetProcAddress('rs2_delete_processing_block', RS2_LibHandle));
    rs2_create_frame_queue := Trs2_create_frame_queue(rs2_GetProcAddress('rs2_create_frame_queue', RS2_LibHandle));
    rs2_delete_frame_queue := Trs2_delete_frame_queue(rs2_GetProcAddress('rs2_delete_frame_queue', RS2_LibHandle));
    rs2_wait_for_frame := Trs2_wait_for_frame(rs2_GetProcAddress('rs2_wait_for_frame', RS2_LibHandle));
    rs2_poll_for_frame := Trs2_poll_for_frame(rs2_GetProcAddress('rs2_poll_for_frame', RS2_LibHandle));
    rs2_try_wait_for_frame := Trs2_try_wait_for_frame(rs2_GetProcAddress('rs2_try_wait_for_frame', RS2_LibHandle));
    rs2_enqueue_frame := Trs2_enqueue_frame(rs2_GetProcAddress('rs2_enqueue_frame', RS2_LibHandle));
    rs2_create_align := Trs2_create_align(rs2_GetProcAddress('rs2_create_align', RS2_LibHandle));
    rs2_create_decimation_filter_block := Trs2_create_decimation_filter_block(rs2_GetProcAddress('rs2_create_decimation_filter_block', RS2_LibHandle));
    rs2_create_temporal_filter_block := Trs2_create_temporal_filter_block(rs2_GetProcAddress('rs2_create_temporal_filter_block', RS2_LibHandle));
    rs2_create_spatial_filter_block := Trs2_create_spatial_filter_block(rs2_GetProcAddress('rs2_create_spatial_filter_block', RS2_LibHandle));
    rs2_create_disparity_transform_block := Trs2_create_disparity_transform_block(rs2_GetProcAddress('rs2_create_disparity_transform_block', RS2_LibHandle));
    rs2_create_hole_filling_filter_block := Trs2_create_hole_filling_filter_block(rs2_GetProcAddress('rs2_create_hole_filling_filter_block', RS2_LibHandle));
    rs2_create_rates_printer_block := Trs2_create_rates_printer_block(rs2_GetProcAddress('rs2_create_rates_printer_block', RS2_LibHandle));
    rs2_create_zero_order_invalidation_block := Trs2_create_zero_order_invalidation_block(rs2_GetProcAddress('rs2_create_zero_order_invalidation_block', RS2_LibHandle));
    rs2_create_huffman_depth_decompress_block := Trs2_create_huffman_depth_decompress_block(rs2_GetProcAddress('rs2_create_huffman_depth_decompress_block', RS2_LibHandle));
    rs2_create_hdr_merge_processing_block := Trs2_create_hdr_merge_processing_block(rs2_GetProcAddress('rs2_create_hdr_merge_processing_block', RS2_LibHandle));
    rs2_create_sequence_id_filter := Trs2_create_sequence_id_filter(rs2_GetProcAddress('rs2_create_sequence_id_filter', RS2_LibHandle));
    rs2_get_processing_block_info := Trs2_get_processing_block_info(rs2_GetProcAddress('rs2_get_processing_block_info', RS2_LibHandle));
    rs2_supports_processing_block_info := Trs2_supports_processing_block_info(rs2_GetProcAddress('rs2_supports_processing_block_info', RS2_LibHandle));
    rs2_is_processing_block_extendable_to := Trs2_is_processing_block_extendable_to(rs2_GetProcAddress('rs2_is_processing_block_extendable_to', RS2_LibHandle));

    rs2_playback_status_to_string := Trs2_playback_status_to_string(rs2_GetProcAddress('rs2_playback_status_to_string', RS2_LibHandle));
    rs2_create_record_device := Trs2_create_record_device(rs2_GetProcAddress('rs2_create_record_device', RS2_LibHandle));
    rs2_create_record_device_ex := Trs2_create_record_device_ex(rs2_GetProcAddress('rs2_create_record_device_ex', RS2_LibHandle));
    rs2_record_device_pause := Trs2_record_device_pause(rs2_GetProcAddress('rs2_record_device_pause', RS2_LibHandle));
    rs2_record_device_resume := Trs2_record_device_resume(rs2_GetProcAddress('rs2_record_device_resume', RS2_LibHandle));
    rs2_record_device_filename := Trs2_record_device_filename(rs2_GetProcAddress('rs2_record_device_filename', RS2_LibHandle));
    rs2_create_playback_device := Trs2_create_playback_device(rs2_GetProcAddress('rs2_create_playback_device', RS2_LibHandle));
    rs2_playback_device_get_file_path := Trs2_playback_device_get_file_path(rs2_GetProcAddress('rs2_playback_device_get_file_path', RS2_LibHandle));
    rs2_playback_get_duration := Trs2_playback_get_duration(rs2_GetProcAddress('rs2_playback_get_duration', RS2_LibHandle));
    rs2_playback_seek := Trs2_playback_seek(rs2_GetProcAddress('rs2_playback_seek', RS2_LibHandle));
    rs2_playback_get_position := Trs2_playback_get_position(rs2_GetProcAddress('rs2_playback_get_position', RS2_LibHandle));
    rs2_playback_device_resume := Trs2_playback_device_resume(rs2_GetProcAddress('rs2_playback_device_resume', RS2_LibHandle));
    rs2_playback_device_pause := Trs2_playback_device_pause(rs2_GetProcAddress('rs2_playback_device_pause', RS2_LibHandle));
    rs2_playback_device_set_real_time := Trs2_playback_device_set_real_time(rs2_GetProcAddress('rs2_playback_device_set_real_time', RS2_LibHandle));
    rs2_playback_device_is_real_time := Trs2_playback_device_is_real_time(rs2_GetProcAddress('rs2_playback_device_is_real_time', RS2_LibHandle));
    rs2_playback_device_set_status_changed_callback := Trs2_playback_device_set_status_changed_callback(rs2_GetProcAddress('rs2_playback_device_set_status_changed_callback', RS2_LibHandle));
    rs2_playback_device_get_current_status := Trs2_playback_device_get_current_status(rs2_GetProcAddress('rs2_playback_device_get_current_status', RS2_LibHandle));
    rs2_playback_device_set_playback_speed := Trs2_playback_device_set_playback_speed(rs2_GetProcAddress('rs2_playback_device_set_playback_speed', RS2_LibHandle));
    rs2_playback_device_stop := Trs2_playback_device_stop(rs2_GetProcAddress('rs2_playback_device_stop', RS2_LibHandle));


    rs2_camera_info_to_string := Trs2_camera_info_to_string(rs2_GetProcAddress('rs2_camera_info_to_string', RS2_LibHandle));
    rs2_stream_to_string := Trs2_stream_to_string(rs2_GetProcAddress('rs2_stream_to_string', RS2_LibHandle));
    rs2_format_to_string := Trs2_format_to_string(rs2_GetProcAddress('rs2_format_to_string', RS2_LibHandle));
    rs2_delete_sensor_list := Trs2_delete_sensor_list(rs2_GetProcAddress('rs2_delete_sensor_list', RS2_LibHandle));
    rs2_get_sensors_count := Trs2_get_sensors_count(rs2_GetProcAddress('rs2_get_sensors_count', RS2_LibHandle));
    rs2_delete_sensor := Trs2_delete_sensor(rs2_GetProcAddress('rs2_delete_sensor', RS2_LibHandle));
    rs2_create_sensor := Trs2_create_sensor(rs2_GetProcAddress('rs2_create_sensor', RS2_LibHandle));
    rs2_create_device_from_sensor := Trs2_create_device_from_sensor(rs2_GetProcAddress('rs2_create_device_from_sensor', RS2_LibHandle));
    rs2_get_sensor_info := Trs2_get_sensor_info(rs2_GetProcAddress('rs2_get_sensor_info', RS2_LibHandle));
    rs2_supports_sensor_info := Trs2_supports_sensor_info(rs2_GetProcAddress('rs2_supports_sensor_info', RS2_LibHandle));
    rs2_is_sensor_extendable_to := Trs2_is_sensor_extendable_to(rs2_GetProcAddress('rs2_is_sensor_extendable_to', RS2_LibHandle));
    rs2_get_depth_scale := Trs2_get_depth_scale(rs2_GetProcAddress('rs2_get_depth_scale', RS2_LibHandle));
    rs2_depth_stereo_frame_get_baseline := Trs2_depth_stereo_frame_get_baseline(rs2_GetProcAddress('rs2_depth_stereo_frame_get_baseline', RS2_LibHandle));
    rs2_get_stereo_baseline := Trs2_get_stereo_baseline(rs2_GetProcAddress('rs2_get_stereo_baseline', RS2_LibHandle));
    rs2_set_region_of_interest := Trs2_set_region_of_interest(rs2_GetProcAddress('rs2_set_region_of_interest', RS2_LibHandle));
    rs2_get_region_of_interest := Trs2_get_region_of_interest(rs2_GetProcAddress('rs2_get_region_of_interest', RS2_LibHandle));
    rs2_open := Trs2_open(rs2_GetProcAddress('rs2_open', RS2_LibHandle));
    rs2_open_multiple := Trs2_open_multiple(rs2_GetProcAddress('rs2_open_multiple', RS2_LibHandle));
    rs2_close := Trs2_close(rs2_GetProcAddress('rs2_close', RS2_LibHandle));
    rs2_start := Trs2_start(rs2_GetProcAddress('rs2_start', RS2_LibHandle));
    rs2_start_cpp := Trs2_start_cpp(rs2_GetProcAddress('rs2_start_cpp', RS2_LibHandle));
    rs2_start_queue := Trs2_start_queue(rs2_GetProcAddress('rs2_start_queue', RS2_LibHandle));
    rs2_stop := Trs2_stop(rs2_GetProcAddress('rs2_stop', RS2_LibHandle));
    rs2_set_notifications_callback := Trs2_set_notifications_callback(rs2_GetProcAddress('rs2_set_notifications_callback', RS2_LibHandle));
    rs2_set_notifications_callback_cpp := Trs2_set_notifications_callback_cpp(rs2_GetProcAddress('rs2_set_notifications_callback_cpp', RS2_LibHandle));
    rs2_get_notification_description := Trs2_get_notification_description(rs2_GetProcAddress('rs2_get_notification_description', RS2_LibHandle));
    rs2_get_notification_timestamp := Trs2_get_notification_timestamp(rs2_GetProcAddress('rs2_get_notification_timestamp', RS2_LibHandle));
    rs2_get_notification_severity := Trs2_get_notification_severity(rs2_GetProcAddress('rs2_get_notification_severity', RS2_LibHandle));
    rs2_get_notification_category := Trs2_get_notification_category(rs2_GetProcAddress('rs2_get_notification_category', RS2_LibHandle));
    rs2_get_notification_serialized_data := Trs2_get_notification_serialized_data(rs2_GetProcAddress('rs2_get_notification_serialized_data', RS2_LibHandle));
    rs2_get_stream_profiles := Trs2_get_stream_profiles(rs2_GetProcAddress('rs2_get_stream_profiles', RS2_LibHandle));
    rs2_get_active_streams := Trs2_get_active_streams(rs2_GetProcAddress('rs2_get_active_streams', RS2_LibHandle));
    rs2_get_stream_profile := Trs2_get_stream_profile(rs2_GetProcAddress('rs2_get_stream_profile', RS2_LibHandle));
    rs2_get_stream_profile_data := Trs2_get_stream_profile_data(rs2_GetProcAddress('rs2_get_stream_profile_data', RS2_LibHandle));
    rs2_set_stream_profile_data := Trs2_set_stream_profile_data(rs2_GetProcAddress('rs2_set_stream_profile_data', RS2_LibHandle));
    rs2_clone_stream_profile := Trs2_clone_stream_profile(rs2_GetProcAddress('rs2_clone_stream_profile', RS2_LibHandle));
    rs2_clone_video_stream_profile := Trs2_clone_video_stream_profile(rs2_GetProcAddress('rs2_clone_video_stream_profile', RS2_LibHandle));
    rs2_delete_stream_profile := Trs2_delete_stream_profile(rs2_GetProcAddress('rs2_delete_stream_profile', RS2_LibHandle));
    rs2_stream_profile_is := Trs2_stream_profile_is(rs2_GetProcAddress('rs2_stream_profile_is', RS2_LibHandle));
    rs2_get_video_stream_resolution := Trs2_get_video_stream_resolution(rs2_GetProcAddress('rs2_get_video_stream_resolution', RS2_LibHandle));
    rs2_get_motion_intrinsics := Trs2_get_motion_intrinsics(rs2_GetProcAddress('rs2_get_motion_intrinsics', RS2_LibHandle));
    rs2_is_stream_profile_default := Trs2_is_stream_profile_default(rs2_GetProcAddress('rs2_is_stream_profile_default', RS2_LibHandle));
    rs2_get_stream_profiles_count := Trs2_get_stream_profiles_count(rs2_GetProcAddress('rs2_get_stream_profiles_count', RS2_LibHandle));
    rs2_delete_stream_profiles_list := Trs2_delete_stream_profiles_list(rs2_GetProcAddress('rs2_delete_stream_profiles_list', RS2_LibHandle));
    rs2_get_extrinsics := Trs2_get_extrinsics(rs2_GetProcAddress('rs2_get_extrinsics', RS2_LibHandle));
    rs2_register_extrinsics := Trs2_register_extrinsics(rs2_GetProcAddress('rs2_register_extrinsics', RS2_LibHandle));
    rs2_override_extrinsics := Trs2_override_extrinsics(rs2_GetProcAddress('rs2_override_extrinsics', RS2_LibHandle));
    rs2_get_video_stream_intrinsics := Trs2_get_video_stream_intrinsics(rs2_GetProcAddress('rs2_get_video_stream_intrinsics', RS2_LibHandle));
    rs2_get_recommended_processing_blocks := Trs2_get_recommended_processing_blocks(rs2_GetProcAddress('rs2_get_recommended_processing_blocks', RS2_LibHandle));
    rs2_get_processing_block := Trs2_get_processing_block(rs2_GetProcAddress('rs2_get_processing_block', RS2_LibHandle));
    rs2_get_recommended_processing_blocks_count := Trs2_get_recommended_processing_blocks_count(rs2_GetProcAddress('rs2_get_recommended_processing_blocks_count', RS2_LibHandle));
    rs2_delete_recommended_processing_blocks := Trs2_delete_recommended_processing_blocks(rs2_GetProcAddress('rs2_delete_recommended_processing_blocks', RS2_LibHandle));
    rs2_import_localization_map := Trs2_import_localization_map(rs2_GetProcAddress('rs2_import_localization_map', RS2_LibHandle));
    rs2_export_localization_map := Trs2_export_localization_map(rs2_GetProcAddress('rs2_export_localization_map', RS2_LibHandle));
    rs2_set_static_node := Trs2_set_static_node(rs2_GetProcAddress('rs2_set_static_node', RS2_LibHandle));
    rs2_get_static_node := Trs2_get_static_node(rs2_GetProcAddress('rs2_get_static_node', RS2_LibHandle));
    rs2_remove_static_node := Trs2_remove_static_node(rs2_GetProcAddress('rs2_remove_static_node', RS2_LibHandle));
    rs2_load_wheel_odometry_config := Trs2_load_wheel_odometry_config(rs2_GetProcAddress('rs2_load_wheel_odometry_config', RS2_LibHandle));
    rs2_send_wheel_odometry := Trs2_send_wheel_odometry(rs2_GetProcAddress('rs2_send_wheel_odometry', RS2_LibHandle));
    rs2_set_intrinsics := Trs2_set_intrinsics(rs2_GetProcAddress('rs2_set_intrinsics', RS2_LibHandle));
    rs2_override_intrinsics := Trs2_override_intrinsics(rs2_GetProcAddress('rs2_override_intrinsics', RS2_LibHandle));
    rs2_set_extrinsics := Trs2_set_extrinsics(rs2_GetProcAddress('rs2_set_extrinsics', RS2_LibHandle));
    rs2_get_dsm_params := Trs2_get_dsm_params(rs2_GetProcAddress('rs2_get_dsm_params', RS2_LibHandle));
    rs2_override_dsm_params := Trs2_override_dsm_params(rs2_GetProcAddress('rs2_override_dsm_params', RS2_LibHandle));
    rs2_reset_sensor_calibration := Trs2_reset_sensor_calibration(rs2_GetProcAddress('rs2_reset_sensor_calibration', RS2_LibHandle));
    rs2_set_motion_device_intrinsics := Trs2_set_motion_device_intrinsics(rs2_GetProcAddress('rs2_set_motion_device_intrinsics', RS2_LibHandle));
    rs2_get_raw_data_size := Trs2_get_raw_data_size(rs2_GetProcAddress('rs2_get_raw_data_size', RS2_LibHandle));
    rs2_delete_raw_data := Trs2_delete_raw_data(rs2_GetProcAddress('rs2_delete_raw_data', RS2_LibHandle));
    rs2_get_raw_data := Trs2_get_raw_data(rs2_GetProcAddress('rs2_get_raw_data', RS2_LibHandle));
    rs2_get_api_version := Trs2_get_api_version(rs2_GetProcAddress('rs2_get_api_version', RS2_LibHandle));
    rs2_log_to_console := Trs2_log_to_console(rs2_GetProcAddress('rs2_log_to_console', RS2_LibHandle));
    rs2_log_to_file := Trs2_log_to_file(rs2_GetProcAddress('rs2_log_to_file', RS2_LibHandle));
    rs2_log_to_callback_cpp := Trs2_log_to_callback_cpp(rs2_GetProcAddress('rs2_log_to_callback_cpp', RS2_LibHandle));
    rs2_log_to_callback := Trs2_log_to_callback(rs2_GetProcAddress('rs2_log_to_callback', RS2_LibHandle));
    rs2_get_log_message_line_number := Trs2_get_log_message_line_number(rs2_GetProcAddress('rs2_get_log_message_line_number', RS2_LibHandle));
    rs2_get_log_message_filename := Trs2_get_log_message_filename(rs2_GetProcAddress('rs2_get_log_message_filename', RS2_LibHandle));
    rs2_get_raw_log_message := Trs2_get_raw_log_message(rs2_GetProcAddress('rs2_get_raw_log_message', RS2_LibHandle));
    rs2_get_full_log_message := Trs2_get_full_log_message(rs2_GetProcAddress('rs2_get_full_log_message', RS2_LibHandle));
    rs2_log := Trs2_log(rs2_GetProcAddress('rs2_log', RS2_LibHandle));
    rs2_depth_frame_get_distance := Trs2_depth_frame_get_distance(rs2_GetProcAddress('rs2_depth_frame_get_distance', RS2_LibHandle));
    rs2_get_time := Trs2_get_time(rs2_GetProcAddress('rs2_get_time', RS2_LibHandle));

    rs2_create_config := Trs2_create_config(rs2_GetProcAddress('rs2_create_config', RS2_LibHandle));
    rs2_delete_config := Trs2_delete_config(rs2_GetProcAddress('rs2_delete_config', RS2_LibHandle));
    rs2_config_enable_stream := Trs2_config_enable_stream(rs2_GetProcAddress('rs2_config_enable_stream', RS2_LibHandle));
    rs2_config_enable_all_stream := rs2_config_enable_all_stream(rs2_GetProcAddress('rs2_config_enable_all_stream', RS2_LibHandle));
    rs2_config_enable_device := Trs2_config_enable_device(rs2_GetProcAddress('rs2_config_enable_device', RS2_LibHandle));
    rs2_config_enable_device_from_file := Trs2_config_enable_device_from_file(rs2_GetProcAddress('rs2_config_enable_device_from_file', RS2_LibHandle));
    rs2_config_enable_device_from_file_repeat_option := rs2_config_enable_device_from_file_repeat_option(rs2_GetProcAddress('rs2_config_enable_device_from_file_repeat_option', RS2_LibHandle));
    rs2_config_enable_record_to_file := rs2_config_enable_record_to_file(rs2_GetProcAddress('rs2_config_enable_record_to_file', RS2_LibHandle));
    rs2_config_disable_stream := Trs2_config_disable_stream(rs2_GetProcAddress('rs2_config_disable_stream', RS2_LibHandle));
    rs2_config_disable_indexed_stream := Trs2_config_disable_indexed_stream(rs2_GetProcAddress('rs2_config_disable_indexed_stream', RS2_LibHandle));
    rs2_config_disable_all_streams := Trs2_config_disable_all_streams(rs2_GetProcAddress('rs2_config_disable_all_streams', RS2_LibHandle));
    rs2_config_resolve := Trs2_config_resolve(rs2_GetProcAddress('rs2_config_resolve', RS2_LibHandle));
    rs2_config_can_resolve := Trs2_config_can_resolve(rs2_GetProcAddress('rs2_config_can_resolve', RS2_LibHandle));

    rs2_create_pipeline := Trs2_create_pipeline(rs2_GetProcAddress('rs2_create_pipeline', RS2_LibHandle));
    rs2_pipeline_stop := Trs2_pipeline_stop(rs2_GetProcAddress('rs2_pipeline_stop', RS2_LibHandle));
    rs2_pipeline_wait_for_frames := Trs2_pipeline_wait_for_frames(rs2_GetProcAddress('rs2_pipeline_wait_for_frames', RS2_LibHandle));
    rs2_pipeline_poll_for_frames := Trs2_pipeline_poll_for_frames(rs2_GetProcAddress('rs2_pipeline_poll_for_frames', RS2_LibHandle));
    rs2_pipeline_try_wait_for_frames := Trs2_pipeline_try_wait_for_frames(rs2_GetProcAddress('rs2_pipeline_try_wait_for_frames', RS2_LibHandle));
    rs2_delete_pipeline := Trs2_delete_pipeline(rs2_GetProcAddress('rs2_delete_pipeline', RS2_LibHandle));
    rs2_pipeline_start := Trs2_pipeline_start(rs2_GetProcAddress('rs2_pipeline_start', RS2_LibHandle));
    rs2_pipeline_start_with_config := Trs2_pipeline_start_with_config(rs2_GetProcAddress('rs2_pipeline_start_with_config', RS2_LibHandle));
    rs2_pipeline_start_with_callback := Trs2_pipeline_start_with_callback(rs2_GetProcAddress('rs2_pipeline_start_with_callback', RS2_LibHandle));
    rs2_pipeline_start_with_callback_cpp := Trs2_pipeline_start_with_callback_cpp(rs2_GetProcAddress('rs2_pipeline_start_with_callback_cpp', RS2_LibHandle));
    rs2_pipeline_start_with_config_and_callback := Trs2_pipeline_start_with_config_and_callback(rs2_GetProcAddress('rs2_pipeline_start_with_config_and_callback', RS2_LibHandle));
    rs2_pipeline_start_with_config_and_callback_cpp := Trs2_pipeline_start_with_config_and_callback_cpp(rs2_GetProcAddress('rs2_pipeline_start_with_config_and_callback_cpp', RS2_LibHandle));
    rs2_pipeline_get_active_profile := Trs2_pipeline_get_active_profile(rs2_GetProcAddress('rs2_pipeline_get_active_profile', RS2_LibHandle));
    rs2_pipeline_profile_get_device := Trs2_pipeline_profile_get_device(rs2_GetProcAddress('rs2_pipeline_profile_get_device', RS2_LibHandle));
    rs2_pipeline_profile_get_streams := Trs2_pipeline_profile_get_streams(rs2_GetProcAddress('rs2_pipeline_profile_get_streams', RS2_LibHandle));
    rs2_delete_pipeline_profile := Trs2_delete_pipeline_profile(rs2_GetProcAddress('rs2_delete_pipeline_profile', RS2_LibHandle));

    Result := True;
  end;

end;



end.



















