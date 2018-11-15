import numpy as np
import cv2
import scipy.io


" ---------- PREAMBLE ---------- "


# COLORS
blue = (255, 0, 0)
green = (0, 255, 0)
red = (0, 0, 255)
yellow = (0, 255, 255)
lower_green = np.array([0, 100, 0])   # B G R
upper_green = np.array([140, 255, 160])


# RESIZE + MASK
def process_frame(frame):
    frame = frame[0:video_height, left:(video_width - right)]
    mask = cv2.inRange(frame, lower_green, upper_green)
    frame_m = cv2.bitwise_and(frame, frame, mask=mask)
    frame_g = cv2.cvtColor(frame_m, cv2.COLOR_RGB2GRAY)
    return frame, frame_m, frame_g


# PLOT POINT
def plot_cg(frame, p, color):
    p1 = p[1, 0]
    p2 = p[0, 0]
    p3 = p[2, 0]
    v_12 = p2 - p1
    v_13 = p3 - p1
    v_cg = (v_12 + v_13) / 2
    p_cg = p1 + v_cg
    cv2.circle(img=frame,
               center=(p_cg[0], p_cg[1]),
               radius=10,
               color=color,
               thickness=-1)


def plot_markers(frame, p, color):
    for k in range(p.shape[0]):
        cv2.circle(img=frame,
                   center=(p[k, 0, 0], p[k, 0, 1]),
                   radius=10,
                   color=color,
                   thickness=-1)


# HOMOGRAPHY
homography_matrix = scipy.io.loadmat('external_H.mat')
H_t = homography_matrix['H']


# PROJECT

def project(p):
    q = np.zeros((p.shape[0], 2))
    for k in range(p.shape[0]):
        q[k] = homography(p[k, 0])
    return q


def homography(p):
    u = p[0]
    v = p[1]
    p_d = np.array([[u],
                    [v],
                    [1]])
    q_d = np.dot(H_t, p_d)
    q = np.array((q_d[0] / q_d[2], q_d[1] / q_d[2]))
    return q[0], q[1]


# CG LOCATION + YAW

def angle(v1, v2):
    dot = v1[0]*v2[0] + v1[1]*v2[1]
    det = v1[0]*v2[1] - v1[1]*v2[0]
    return np.arctan2(det, dot)


def cg_states(q1, q2, q3):
    v_12 = q2 - q1
    v_13 = q3 - q1
    v_cg = (v_12 + v_13) / 2
    q_cg = q1 + v_cg
    v_0 = np.array((1, 0))
    psi_cg = -1*angle(v_cg, v_0)
    return q_cg, psi_cg


# SELECT FILE + CROP LEFT ARGUMENT
cap = cv2.VideoCapture('external_raw.mov')
video_width = int(cap.get(3))
video_height = int(cap.get(4))
fps = cap.get(5)
left = 300
right = 100

# CREATE VIDEO EXPORT
codec = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
tracked = cv2.VideoWriter()
tracked.open('external_tracked.avi', codec, fps, (video_width-left-right, video_height), 1)

masked = cv2.VideoWriter()
masked.open('external_color_filter.avi', codec, fps, (video_width-left-right, video_height), 1)


" ---------- ALGORITHM STARTS HERE ---------- "


# FETCH INITIAL FRAME
_, frame_0 = cap.read()
frame_0, frame_0m, frame_0g = process_frame(frame_0)


# DETECT MARKERS
p_0 = cv2.goodFeaturesToTrack(image=frame_0g,
                              maxCorners=3,
                              qualityLevel=0.25,
                              minDistance=80,
                              mask=None)


# CG STATES
q_0 = project(p_0)
q_cg_0, psi_cg_0 = cg_states(q_0[1], q_0[0], q_0[2])
x_CG = q_cg_0[0]
y_CG = q_cg_0[1]
psi_CG = psi_cg_0





" ---------- LOOP STARTS HERE ---------- "


# LOOP THROUGH FRAMES
while cap.isOpened():

    ret, frame_i = cap.read()

    if ret == True:

        # FETCH
        frame_i, frame_im, frame_ig = process_frame(frame_i)

        # TRACK MARKERS
        p_i, status, error = cv2.calcOpticalFlowPyrLK(prevImg=frame_0g,
                                                      nextImg=frame_ig,
                                                      prevPts=p_0,
                                                      nextPts=None,
                                                      winSize=(100, 100),
                                                      maxLevel=3,
                                                      criteria=(cv2.TERM_CRITERIA_EPS |
                                                                cv2.TERM_CRITERIA_COUNT,
                                                                100, 0.01))

        if status.any() == 0:
            cap.release()
            cv2.destroyAllWindows()

        p_0 = p_i

        # CG STATES
        q_i = project(p_i)
        q_cg_i, psi_cg_i = cg_states(q_i[1], q_i[0], q_i[2])
        x_CG_i = q_cg_i[0]
        y_CG_i = q_cg_i[1]
        psi_CG_i = psi_cg_i
        x_CG = np.append(x_CG, x_CG_i)
        y_CG = np.append(y_CG, y_CG_i)
        psi_CG = np.append(psi_CG, psi_CG_i)

        # EXPORT FRAME
        frame_0g = frame_ig
        frame_i_gray = cv2.cvtColor(frame_i, cv2.COLOR_RGB2GRAY)
        frame_i_gray = cv2.cvtColor(frame_i_gray, cv2.COLOR_GRAY2RGB)
        plot_markers(frame_i_gray, p_i, green)
        plot_cg(frame_i_gray, p_i, yellow)
        cv2.imshow('output', frame_i_gray)
        # cv2.waitKey(1)
        tracked.write(frame_i_gray)
        masked.write(frame_im)

    # END LOOP
    else:
        cap.release()
        cv2.destroyAllWindows()

# EXPORT STATES
save = {}
save['x_CG'] = x_CG
save['y_CG'] = y_CG
save['psi_CG'] = psi_CG
scipy.io.savemat('external_tracked.mat', save)
