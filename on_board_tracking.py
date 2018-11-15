import scipy.io
import numpy as np
import cv2
import time

# COLORS
blue = (255, 0, 0)
green = (0, 255, 0)
red = (0, 0, 255)
yellow = (0, 255, 255)
black = (0, 0, 0)
white = (255, 255, 255)


# IMPORT DATA
measurements = scipy.io.loadmat('on_board_sensors.mat')
t = np.array(measurements['t'])
image = np.array(measurements['image'])
I_frames = image.shape[1]


# DISTORTION
distortion = scipy.io.loadmat('dist.mat')
mtx = distortion['mtx']
dist = distortion['dist']
mtx2 = distortion['mtx2']
xd = distortion['xd']
yd = distortion['yd']
wd = distortion['wd']
hd = distortion['hd']
frame_width = wd
frame_height = hd


# HOMOGRAPHY
homography_matrix = scipy.io.loadmat('on_board_H.mat')
H_t = homography_matrix['H']


def un_distort(frame):
    dst = cv2.undistort(frame, mtx, dist, None, mtx2)
    dst = dst[35:455, 7:627]
    return dst


def homography(p):
    u = p[0]
    v = p[1]
    p_d = np.array([[u],
                    [v],
                    [1]])
    q_d = np.dot(H_t, p_d)
    q = np.array((q_d[0] / q_d[2], q_d[1] / q_d[2]))
    return q[0], q[1]


def project(p):
    q = np.zeros((p.shape[0], 2))
    for k in range(p.shape[0]):
        q[k] = homography(p[k, 0])
    return q

# THRESHOLD
lower_red = np.array([0, 0, 75])
upper_red = np.array([175, 75, 255])

# DETECTION
N_p = 8
D_p = 10
Q_p = 0.001

# TRACKING
p_w = 25
L_pyr = 3
N_i_max = 1
epsilon = 0.001

# ROI
ROI = np.zeros([420, 620], np.uint8)
pts = np.array([[50, 350], [620, 350], [620, 50],  [25, 50]], np.int32)
pts = pts.reshape((-1, 1, 2))
cv2.fillPoly(ROI, [pts], color=white, lineType=8, shift=0)


# APPLY RED THRESHOLD, ROI, GREYSCALE
def process_frame(frame):
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame = un_distort(frame)
    red_threshold = cv2.inRange(frame, lower_red, upper_red)
    frame_red = cv2.bitwise_and(frame, frame, mask=red_threshold)
    frame_roi = cv2.bitwise_and(frame_red, frame_red, mask=det_ROI)
    frame_roi = cv2.cvtColor(frame_roi, cv2.COLOR_RGB2GRAY)
    frame_grey = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    return frame, frame_red, frame_grey, frame_roi


det_ROI = np.zeros([420, 620], np.uint8)
pts = np.array([[0, 420], [100, 420], [190, 320], [515, 320], [620, 420], [620, 60],  [310, 30], [0, 60]], np.int32)
pts = pts.reshape((-1, 1, 2))
cv2.fillPoly(det_ROI, [pts], color=white, lineType=8, shift=0)


# DETECT & TRACK
def detect_track(frame_g0, p_0, frame_gi):

    p_i, status, error = cv2.calcOpticalFlowPyrLK(
                                    prevImg=frame_g0,
                                    nextImg=frame_gi,
                                    prevPts=p_0,
                                    nextPts=None,
                                    winSize=(p_w, p_w),
                                    maxLevel=L_pyr,
                                    criteria=(cv2.TERM_CRITERIA_EPS |
                                              cv2.TERM_CRITERIA_COUNT,
                                              N_i_max, epsilon))
    p_i = p_i.reshape(-1, 1, 2)

    p_0 = cv2.goodFeaturesToTrack(image=frame_gi,
                                  maxCorners=N_p,
                                  qualityLevel=Q_p,
                                  minDistance=D_p,
                                  mask=frame_roi)

    p_0 = p_0.reshape(-1, 1, 2)

    frame_0 = frame_gi

    return p_i, p_0, frame_0


def plot_markers(frame_p, p, color):
    for n in range(p.shape[0]):
        cv2.circle(img=frame_p,
                   center=(p[n, 0, 0], p[n, 0, 1]),
                   radius=4,
                   color=color,
                   thickness=-1)


def plot_velocity(frame_v, p_0, p_i, color):
    for n in range(p_i.shape[0]):
        cv2.line(img=frame_v,
                 pt1=(p_0[n, 0, 0], p_0[n, 0, 1]),
                 pt2=(p_i[n, 0, 0], p_i[n, 0, 1]),
                 color=color,
                 thickness=2,
                 lineType=8,
                 shift=0)


# EMPTY ARRAY
output = np.zeros((I_frames, N_p, 5))


def calc_velocity(p_0, t_0, p_i, t_i):
    dt = t_i - t_0
    q_0 = project(p_0)
    q_i = project(p_i)
    if dt > 0:
        for n in range(q_i.shape[0]):
            output[i, n, 0] = t_i  # t
            output[i, n, 1] = q_i[n, 0]  # x_i
            output[i, n, 2] = q_i[n, 1]  # y_i
            output[i, n, 3] = (q_i[n, 0] - q_0[n, 0]) / dt  # V_x
            output[i, n, 4] = (q_i[n, 1] - q_0[n, 1]) / dt  # V_y


# EXPORT VIDEOS
codec = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
size = (frame_width, frame_height)

on_board_raw = cv2.VideoWriter()
on_board_raw.open('on_board_raw.avi', codec, 30, size, 1)

on_board_mask = cv2.VideoWriter()
on_board_mask.open('on_board_mask.avi', codec, 30, size, 1)

on_board_points = cv2.VideoWriter()
on_board_points.open('on_board_points.avi', codec, 30, size, 1)

on_board_tracked = cv2.VideoWriter()
on_board_tracked.open('on_board_tracked.avi', codec, 30, size, 1)


# INITIAL FRAME
t_0 = t[0, 0]
frame_0, frame_red_0, frame_grey_0, frame_roi = process_frame(image[0, 0])


p_0 = cv2.goodFeaturesToTrack(image=frame_grey_0,
                              maxCorners=N_p,
                              qualityLevel=Q_p,
                              minDistance=D_p,
                              mask=frame_roi)

time_start = time.clock()
# MAIN LOOP
for i in range(I_frames):

    t_i = t[0, i]

    # FETCH
    frame, frame_red, frame_grey, frame_roi = process_frame(image[0, i])
    on_board_raw.write(frame)
    on_board_mask.write(frame_red)

    frame_points = frame
    frame_tracked = frame

    # PREVIOUS DETECTED POINTS
    plot_markers(frame_points, p_0, yellow)
    p_00 = p_0

    # CURRENT TRACKER POINTS
    p_i, p_0, frame_grey_0 = detect_track(frame_grey_0, p_0, frame_grey)

    cv2.imshow('frame', frame_roi)
    cv2.waitKey(1)

    plot_markers(frame_points, p_i, green)
    on_board_points.write(frame_points)

    calc_velocity(p_00, t_0, p_i, t_i)
    plot_velocity(frame_tracked, p_00, p_i, blue)
    on_board_tracked.write(frame_tracked)

    cv2.imshow('frame', frame_tracked)
    # cv2.waitKey(1)

    t_0 = t_i
    # END OF LOOP
print((time.clock() - time_start))
cv2.destroyAllWindows()

# EXPORT MEASUREMENTS
save = {}
save['output'] = output
scipy.io.savemat('on_board_tracked', save)
