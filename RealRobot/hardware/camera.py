import cvb
import cv2
import numpy as np

# Function to discover device
def discover_device():
    # Discover devices
    discovered_devices = cvb.DeviceFactory.discover_from_root(cvb.DiscoverFlags.IgnoreVins)
    if not discovered_devices:
        raise RuntimeError("No devices found.")
    # Select the first discovered device
    device_info = discovered_devices[0]
    return device_info.access_token

# Configure camera settings
def configure_camera(camera):
    try:
        node_map = camera.node_maps["Device"]

        # Set resolution and ROI
        node_map["Width"].value = 2560
        node_map["Height"].value = 2048
        node_map["OffsetX"].value = 0
        node_map["OffsetY"].value = 0

        # Set pixel format
        node_map["PixelFormat"].value = "BayerGR8"

        # Print camera settings
        print("Camera Settings:")
        print(f"  Width: {node_map['Width'].value}")
        print(f"  Height: {node_map['Height'].value}")
        print(f"  Pixel Format: {node_map['PixelFormat'].value}")
    except KeyError as e:
        print(f"Configuration error: {e}. Verify the node names in your camera's GenICam settings.")

# Adjust white balance for the image
# Adjust white balance for the image
def adjust_white_balance(image):
    # Convert the image to float for precise adjustments
    result = image.astype(np.float32)
    # Calculate the mean intensity for each channel
    mean_b = np.mean(result[:, :, 0])
    mean_g = np.mean(result[:, :, 1])
    mean_r = np.mean(result[:, :, 2])

    # Calculate the scaling factors
    mean_gray = (mean_b + mean_g + mean_r) / 3
    scale_b = mean_gray / mean_b
    scale_g = mean_gray / mean_g
    scale_r = mean_gray / mean_r

    # Scale each channel
    result[:, :, 0] *= scale_b
    result[:, :, 1] *= scale_g
    result[:, :, 2] *= scale_r

    # Clip the values to [0, 255] and convert back to uint8
    result = np.clip(result, 0, 255).astype(np.uint8)
    return result


# Detect Aruco markers in a frame
def detect_aruco_tags(image_frame):
    # Define the dictionary and parameters for Aruco detection
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    aruco_params = cv2.aruco.DetectorParameters_create()

    # Detect Aruco markers
    gray_frame = cv2.cvtColor(image_frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = cv2.aruco.detectMarkers(gray_frame, aruco_dict, parameters=aruco_params)

    # Draw detected markers
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(image_frame, corners, ids)
        print(f"Detected Aruco IDs: {ids.flatten()}")

    # Draw rejected markers (optional for debugging)
    if rejected:
        cv2.aruco.drawDetectedMarkers(image_frame, rejected, borderColor=(0, 0, 255))

# Main function
def main():
    try:
        access_token = discover_device()
        print("Device discovered. Attempting to open...")

        # Open the device with the correct acquisition stack
        with cvb.DeviceFactory.open(access_token, cvb.AcquisitionStack.Vin) as camera:
            print("Camera opened successfully.")

            # Configure the camera
            configure_camera(camera)

            # Retrieve and start the stream
            stream = camera.stream()
            stream.start()
            print("Streaming started... Press 'q' to quit.")

            try:
                while True:
                    image, status = stream.wait()
                    if status == cvb.WaitStatus.Ok:
                        # Convert the image to a NumPy array
                        frame = cvb.as_array(image)

                        # Adjust white balance
                        frame = adjust_white_balance(frame)

                        # Detect Aruco markers
                        detect_aruco_tags(frame)

                        # Display the processed frame
                        cv2.imshow("Aruco Detection", frame)

                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break
            finally:
                stream.abort()
                print("Streaming stopped.")
    except RuntimeError as e:
        print(f"Runtime error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        cv2.destroyAllWindows()
        print("Application closed.")

if __name__ == "__main__":
    main()
