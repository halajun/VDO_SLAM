from plotter import CSVLoader
import matplotlib.pyplot as plt

if __name__ == '__main__':
    pnp_camera = CSVLoader("../frame_camera.csv")
    pnp_world = CSVLoader("../frame_world.csv")

    print(len(pnp_camera.rows))
    print(len(pnp_world.rows))


    camera_t_errors = []
    world_t_errors = []
    frames = []
    for camera_error, world_error in zip(pnp_camera.rows, pnp_world.rows):
        t_err_cam = float(camera_error["r_error"])
        t_error_world = float(world_error["r_error"])

        print("{}, {}".format(t_err_cam, t_error_world))

        camera_t_errors.append(t_err_cam)
        world_t_errors.append(t_error_world)

        assert(camera_error["frame_id"] == world_error["frame_id"])
        frames.append(int(camera_error["frame_id"]))

    plt.plot(frames, camera_t_errors, "-b", label="PnP Camera")
    plt.plot(frames, world_t_errors, "-r", label="PnP World")
    plt.title("Pose estimation using PnP in world vs camera frame")
    plt.xlabel("Frames")
    plt.ylabel("Absolute Rotation Error (r)")

    plt.legend(loc="upper left")
    plt.savefig("pnp_world_cam_rot_d1.png")