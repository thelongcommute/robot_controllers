import cv2
import sys
import numpy as np
import glob
import re

def segment_image(img):
    #  START Dont Include in Robot Controller ______
    # img = cv2.imread(args[1])
    #  END Dont Include in Robot Controller ______
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ytop = 148
    ybot = 175
    dx1 = 75
    dx2 = 37
    target_width = dx2
    target_height = ybot-ytop
    dim = (target_width, target_height)
    # Array of shape (6,2,2). Each index contains two arrays defining
    # a character's top left than bottom right corner.
    pix_map = np.array([[[75,5],[130,5+dx1]],[[75,76],[130,76+dx1]],
                        [[ytop,3],[ybot,3+dx2]],[[ytop,33],[ybot,33+dx2]],
                        [[ytop,84],[ybot,84+dx2]],[[ytop,115],[ybot,115+dx2]]])

    #  START Dont Include in Robot Controller ______
    # add unique identifier
    # result = re.search('/(P.*).png', args[1])
    # split = result.group(1)
    # res2 = re.search('(.*)'+split+'.png',args[1])
    # split = split.replace("_","")
    # path = res2.group(1)
    # print("Full Plate {}".format(split))
    #  END Dont Include in Robot Controller ______
    char_imgs = []
    # for Robot Rontroller do this:
    for i in range(1,6):
        top = pix_map[i,0]
        bot = pix_map[i,1]
        char_img = img_gray[top[0]:bot[0],top[1]:bot[1]]
        # Resize Parking ID imgs to have same size as plate characters
        char_img = cv2.resize(char_img, dim, interpolation =cv2.INTER_AREA)
        # set right bin threshold
        avg_val = np.ceil(np.mean(char_img))
        # print("avg_va; {}".format(avg_val))
        threshold = avg_val*0.83
        _, char_img = cv2.threshold(char_img, threshold, 255, cv2.THRESH_BINARY)
        # for Robot Rontroller do thmis:
        char_imgs.append(char_img)

        # return char_imgs

        #  START Dont Include in Robot Controller ______
        # cv2.imshow("char",char_img)
        # cv2.waitKey(0)

        # if i == 1:
        #     path2 = path+"segmented_parkingID/"+split[i] +"_*"+".png"
        # else:
        #     path2 = path+"segmented_plate/"+split[i] +"_*"+".png"

        # num_imgs = len(glob.glob(path2))
        # print("num_imgs: {}".format(num_imgs))
        # filename = path2.replace("*",str(num_imgs+1))
        # cv2.imwrite(filename, char_img)
        # print("Saved image to: " + filename)
        #  END Dont Include in Robot Controller ______
    # for Robot Rontroller do this:
    return char_imgs

if __name__ == '__main__':
    segment_image(sys.argv)