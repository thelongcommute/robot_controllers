import cv2
import sys
import numpy as np

# https://www.pyimagesearch.com/2014/08/25/4-point-opencv-getperspective-transf
# orm-example/
def four_point_transform(image, rect):
    # obtain a consistent order of the points and unpack them
    # individually
    (tl, tr, br, bl) = rect
    # compute the width of the new image, which will be the
    # maximum (euclidean) distance between bottom-right and bottom-left
    # x-coordiates or the top-right and top-left x-coordinates
    widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
    widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
    maxWidth = max(int(widthA), int(widthB))
    # compute the height of the new image, which will be the
    # maximum distance between the top-right and bottom-right
    # y-coordinates or the top-left and bottom-left y-coordinates
    heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
    heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
    maxHeight = max(int(heightA), int(heightB))
    # now that we have the dimensions of the new image, construct
    # the set of destination points to obtain a "birds eye view",
    # (i.e. top-down view) of the image, again specifying points
    # in the top-left, top-right, bottom-right, and bottom-left
    # order
    dst = np.array([
        [0, 0],
        [maxWidth - 1, 0],
        [maxWidth - 1, maxHeight - 1],
        [0, maxHeight - 1]], dtype = "float32")
    # compute the perspective transform matrix and then apply it
    M = cv2.getPerspectiveTransform(rect, dst)
    warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
    # return the warped image
    return warped

def process_image_and_flatten(cv_img, car_side):
    # for Robot Controller add car_side as a parameter for the function

    #  START Dont Include in Robot Controller ______
    orig_img_bgr = cv_img
    #  END Dont Include in Robot Controller ______



    # we will set this as an argument later, when we test imgs on right side of screen too. 0 means left, 1 means right
    t = 10
    orig_hsv = cv2.cvtColor(orig_img_bgr, cv2.COLOR_BGR2HSV)
    # cv2.imshow("orig",orig_img_bgr)
    lower_blue = np.array([120-t,120,0])
    upper_blue = np.array([120+t,255,255])
    mask = cv2.inRange(orig_hsv, lower_blue, upper_blue)
    # cv2.imshow("blue_thresh", mask)
    # cv2.waitKey(0)

    #blur - can also experiment with removing this
    # blur = cv2.blur(mask,(5,5))

    # dilate and erode, can experiment with the ordering of these, as
    # well as kernel size and number of iterations
    kernel = np.ones((6,6),np.uint8)
    erosion = cv2.erode(mask,kernel,iterations = 1)
    # cv2.imshow("erosion", erosion)
    # cv2.waitKey(0)
    dilation = cv2.dilate(erosion,kernel,iterations = 1)
    # cv2.imshow("dilation", dilation)
    # cv2.waitKey(0)

    # binary image
    threshold = 20
    _, img_bin = cv2.threshold(dilation, threshold, 255, cv2.THRESH_BINARY)

    # cv2.imshow("img_bin", img_bin)
    # cv2.waitKey(0)
    # # get left and right edges of plate
    thresh = 255*20
    av_cols = np.sum(img_bin, axis=1)
    av_cols[av_cols<=thresh] = 0
    av_cols[av_cols>thresh] = 1

    # get bottom 30 rows of blue in the image
    non_zero = np.nonzero(av_cols)[0][-40:]

    # get bottom of each side block of blue on left and right side of plate
    thresh = 255*10
    av_rows_bot = np.sum(img_bin[non_zero,:], axis=0)
    av_rows_bot[av_rows_bot<=thresh] = 0
    av_rows_bot[av_rows_bot>thresh] = 1
    # get inner vertical edge, vert edges are the second edge if you
    # count from the edge.
    # Count edges starting from the side specified by car_side
    # TODO: Will have to modify this when we decide at what point to that the img
    vert_edges = np.flatnonzero(av_rows_bot[:-1] != av_rows_bot[1:])
    # print("vert_edges:{}".format(vert_edges))
    # this is so it does not fail if it does not detect 4 edges
    if len(vert_edges) < 4:
        return None
    if car_side == 0:
        inner_vert_edges = vert_edges[[1,2]]
    else:
        inner_vert_edges = vert_edges[-2:]
    # Split image into two, so that we can get the bottom
    # of each blue rectangle
    t = 20
    left_side = img_bin[:,inner_vert_edges[0]-t:inner_vert_edges[0]]
    right_side = img_bin[:,inner_vert_edges[1]:inner_vert_edges[1]+t]

    # get y coords of 4 points of plate
    thresh = 255*t//2
    av_cols_left = np.sum(left_side, axis=1)
    av_cols_right = np.sum(right_side, axis=1)
    left_side_nonzero = np.argwhere(av_cols_left > thresh).ravel()
    right_side_nonzero = np.argwhere(av_cols_right > thresh).ravel()

    top_left_y = left_side_nonzero[0]
    top_right_y = right_side_nonzero[0]
    bot_left_y = left_side_nonzero[-1]
    bot_right_y = right_side_nonzero[-1]

    # points in clockwise order starting from top left (ordered x, y pair)
    rect = np.zeros((4, 2), dtype = "float32")
    rect[0] = np.array([inner_vert_edges[0], top_left_y])
    rect[1] = np.array([inner_vert_edges[1], top_right_y])
    rect[2] = np.array([inner_vert_edges[1], bot_right_y])
    rect[3] = np.array([inner_vert_edges[0], bot_left_y])
    warped = four_point_transform(orig_img_bgr, rect)

    target_width = 160
    target_height = 200
    dim = (target_width, target_height)
    # # resize image
    resized = cv2.resize(warped, dim, interpolation = cv2.INTER_AREA)
    # cv2.imshow("resized", resized)
    # cv2.waitKey(0)
    return resized

    #  START Dont Include in Robot Controller ______
    # filename = args[1].split("/")[-1]
    # file_path = args[1][0:-len(filename)] + "flattened/" + filename
    # cv2.imwrite(file_path, resized)
    # print("Saved image to: " + file_path)
    #  END Dont Include in Robot Controller ______
if __name__ == '__main__':
    process_image_and_flatten(sys.argv)