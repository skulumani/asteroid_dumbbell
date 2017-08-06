import numpy as np 
import cv2
import pdb

STAGE_FIRST_FRAME = 0
STAGE_SECOND_FRAME = 1
STAGE_DEFAULT_FRAME = 2
kMinNumFeature = 1500

lk_params = dict(winSize  = (21, 21), 
                                #maxLevel = 3,
                criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

def featureTracking(image_ref, image_cur, px_ref):
        kp2, st, err = cv2.calcOpticalFlowPyrLK(image_ref, image_cur, px_ref, None, **lk_params)  #shape: [k,2] [k,1] [k,1]

        st = st.reshape(st.shape[0])
        kp1 = px_ref[st == 1]
        kp2 = kp2[st == 1]

        return kp1, kp2


class PinholeCamera:
        def __init__(self, width, height, fx, fy, cx, cy, 
                                k1=0.0, k2=0.0, p1=0.0, p2=0.0, k3=0.0):
                self.width = width
                self.height = height
                self.fx = fx
                self.fy = fy
                self.cx = cx
                self.cy = cy
                self.distortion = (abs(k1) > 0.0000001)
                self.d = [k1, k2, p1, p2, k3]


class VisualOdometry:
        def __init__(self, cam, time, i_state, RT_vector, R_i2bcam_vector):
                self.frame_stage = 0
                self.cam = cam
                self.new_frame = None
                self.last_frame = None

                # these are the camera pose matrices
                self.cur_R = RT_vector[0,:].reshape((3, 4))[:, 0:3]
                self.cur_t = RT_vector[0,:].reshape((3, 4))[:, 3] 
                self.px_ref = None
                self.px_cur = None
                self.focal = cam.fx
                self.pp = (cam.cx, cam.cy)
                self.true_pos = i_state[:, 0:3]
                self.detector = cv2.FastFeatureDetector_create(threshold=25, nonmaxSuppression=True)

        def update_motion_estimate(self, R, t, ii):
            self.trueX, self.trueY, self.trueZ = self.true_pos[ii, 0],self.true_pos[ii, 1],self.true_pos[ii, 2]

            # get scale from data
            scale = np.sqrt( np.sum((self.true_pos[ii-1, :] - self.true_pos[ii, :])**2, axis=0))
            # update motion estimate 
            self.cur_t = self.cur_t +  scale * np.squeeze(self.cur_R.dot(t))
            self.cur_R = R.dot(self.cur_R)


        def processFirstFrame(self):
                self.px_ref = self.detector.detect(self.new_frame)
                self.px_ref = np.array([x.pt for x in self.px_ref], dtype=np.float32)
                self.frame_stage = STAGE_SECOND_FRAME

        def processSecondFrame(self, ii):
                self.px_ref, self.px_cur = featureTracking(self.last_frame, self.new_frame, self.px_ref)
                E, mask = cv2.findEssentialMat(self.px_cur, self.px_ref, focal=self.focal, pp=self.pp, method=cv2.RANSAC, prob=0.999, threshold=1.0)
                _, R, t, mask = cv2.recoverPose(E, self.px_cur, self.px_ref, focal=self.focal, pp = self.pp)

                # update estimate of current position and orientation from the initial known value
                self.update_motion_estimate(R, t, ii)

                self.frame_stage = STAGE_DEFAULT_FRAME 
                self.px_ref = self.px_cur

        def processFrame(self, ii):
                self.px_ref, self.px_cur = featureTracking(self.last_frame, self.new_frame, self.px_ref)
                E, mask = cv2.findEssentialMat(self.px_cur, self.px_ref, focal=self.focal, pp=self.pp, method=cv2.RANSAC, prob=0.999, threshold=1.0)
                _, R, t, mask = cv2.recoverPose(E, self.px_cur, self.px_ref, focal=self.focal, pp = self.pp)
                self.update_motion_estimate(R, t, ii)

                if(self.px_ref.shape[0] < kMinNumFeature):
                        self.px_cur = self.detector.detect(self.new_frame)
                        self.px_cur = np.array([x.pt for x in self.px_cur], dtype=np.float32)
                self.px_ref = self.px_cur

        def update(self, img, ii):
                assert(img.ndim==2 and img.shape[0]==self.cam.height and img.shape[1]==self.cam.width), "Frame: provided image has not the same size as the camera model or image is not grayscale"
                self.new_frame = img

                if(self.frame_stage == STAGE_DEFAULT_FRAME):
                        self.processFrame(ii)
                elif(self.frame_stage == STAGE_SECOND_FRAME):
                        self.processSecondFrame(ii)
                elif(self.frame_stage == STAGE_FIRST_FRAME):
                        self.processFirstFrame()
                self.last_frame = self.new_frame
