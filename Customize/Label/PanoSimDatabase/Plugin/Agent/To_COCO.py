import json
import glob
import cv2 as cv
import os
from Demo_FreeDriving_Source import CameraTools
import pandas as pd
import csv
import numpy as np
import re
import pickle

class tococo(object):
    def __init__(self, jpg_paths, label_path, save_path):
        self.images = []
        self.categories = []
        self.annotations = []
        self.jpgpaths = jpg_paths
        self.save_path = save_path
        self.label_path = label_path
        self.class_ids = {'pos': 1}
        self.class_id = 1
        self.class_name = "car"
        self.category_dic = {0:'unlabeled',1:'ego_vehicle',2:'car', 3:'truck', 4:'bus', 5:'train', 6:'other_large_vehicle', 7:'static_small_vehicle',
                             8:'rider', 9:'pedestrian', 10:'dog', 11:'pole', 12:'movable_barrier', 13:'traffic_sign',
                             14:'traffic_light',15:'building',16:'vegetation',17:'road', 18:'sidewalk',19:'parking', 20:'ground',
                             21:'road_marker', 22: 'sky', 23:'dynamic', 24:'static'}
        self.super_category = ['unlabeled', 'vehicle', 'cyclist', 'pedestrian', 'animal', 'obstruction','traffic_facility', 'misc']
        self.coco = {}
        self.weather = 'sunny'
        self.time = 'morning'
        self.picture_format = "jpg"
        self.label_name = "MonoCameraSensor.0"
        self.timestamp = 0
        self.occlusion = 0.0
        self.obj_id = 0
    def npz_to_coco(self):
        annid = 0
        data = []
        file = open(self.label_path, 'rb')
        while True:
            try:
                loaded = pickle.load(file)
                data.append(loaded)
            except EOFError:
                break
        print(len(data))
        for num, jpg_path in enumerate(self.jpgpaths):
            self.label_name = self.label_path.split('\\')[-1].split('.')[0]
            self.label_name = self.label_name.split('_')[-1]
            imgname = jpg_path.split('\\')[-1].split('.')[0]
            timestamp = int(imgname.split('_')[-1])

            self.picture_format = jpg_path.split('\\')[-1].split('.')[-1]
            img = cv.imread(jpg_path)
            h, w = img.shape[:-1]
            self.images.append(self.get_images(imgname, h, w, num))

            if int(timestamp) >= 60100:
                continue
            frames = int(int(timestamp)/100) - 6
            if frames >= len(data):
                frames = len(data) - 1

            data_list = data[frames]
            newlist = []
            if len(data_list) == 0:
                continue
            for kl in range(len(data_list)):
                for nn in range(len(data_list[kl])):
                    newlist.append(data_list[kl][nn])

            size = int(len(newlist) / 21)
            for j in range(size):
                px, py, pw, ph = float(newlist[j*21+1]), float(newlist[j*21+2]), float(newlist[j*21+3]), float(newlist[j*21+4])
                box = [px, py, pw, ph]
                self.class_id = int(newlist[j*21])
                self.class_name = self.category_dic[self.class_id]
                vertex_list = [float(newlist[j*21+5]), float(newlist[j*21+6]), float(newlist[j*21+7]), float(newlist[j*21+8]),
                          float(newlist[j*21+9]), float(newlist[j*21+10]), float(newlist[j*21+11]), float(newlist[j*21+12]),float(newlist[j*21+13])]
                self.timestamp = int(newlist[j*21+14])
                self.occlusion = float(newlist[j*21+15])
                self.obj_id = int(newlist[j*21+20])
                quaternion = [float(newlist[j*21+16]), float(newlist[j*21+17]), float(newlist[j*21+18]), float(newlist[j*21+19])]
                self.annotations.append(self.get_annotations(box, num, annid, self.class_id, vertex_list, self.timestamp,self.occlusion, quaternion, self.obj_id))
                annid = annid + 1

        self.coco["images"] = self.images
        for p in range(len(self.category_dic)):
            category = {}
            if p == 0:
                category["supercategory"] = self.super_category[0]
            elif p > 0 and p < 8:
                category["supercategory"] = self.super_category[1]
            elif p == 8:
                category["supercategory"] = self.super_category[2]
            elif p == 9:
                category["supercategory"] = self.super_category[3]
            elif p == 10:
                category["supercategory"] = self.super_category[4]
            elif p > 10 and p < 13:
                category["supercategory"] = self.super_category[5]
            elif p >= 13 and p < 22:
                category["supercategory"] = self.super_category[6]
            else:
                category["supercategory"] = self.super_category[7]
            category['id'] = p
            category['name'] = self.category_dic[p]
            self.categories.append(category)
        self.coco["categories"] = self.categories
        self.coco["annotations"] = self.annotations
        # print(self.coco)

    def get_images(self, filename, height, width, image_id):
        image = {}
        image["height"] = height
        image['width'] = width
        image["id"] = image_id

        image["file_name"] = filename + '.' + self.picture_format
        image["period"] = self.time
        image["weather"] = self.weather
        return image

    def get_categories(self, name, class_id):
        category = {}
        category["supercategory"] = "vehicle"
        category['id'] = 1
        category['name'] = name
        return category

    def get_annotations(self, box, image_id, ann_id, class_id, vertex_list, timestamp, occlusion, qua, obj_id):
        annotation = {}
        w, h = box[2], box[3]
        area = w * h
        annotation['iscrowd'] = 0
        annotation['corner_case'] = False
        annotation['image_id'] = image_id
        annotation['bbox'] = box
        annotation['area'] = float(area)
        annotation['category_id'] = class_id
        annotation['id'] = ann_id
        annotation['3dbbox'] = vertex_list
        annotation['orientation_angle'] = qua
        annotation['timestamp'] = timestamp
        annotation['occlusion_ratio'] = occlusion
        annotation['object_id'] = obj_id
        return annotation

    def save_json(self):
        self.npz_to_coco()
        label_dic = self.coco
        instances= json.dumps(label_dic)
        f = open(os.path.join(save_path + '/' + self.label_name +'.json'), 'w')
        f.write(instances)
        f.close()



label_path = 'C:\PanoSimDatabase\Experiment\HowTo_Save_Label\Temp\Data\ObjectPerception.0\HowTo_Save_Label_Front.txt'
file_path = 'C:\PanoSimDatabase\Experiment\HowTo_Save_Label\Temp\Data\MonoCameraSensor.0/*.png'
jpg_paths = glob.glob(file_path)
save_path = r'D:\Data\Annotations'
c = tococo(jpg_paths, label_path, save_path)
c.save_json()
