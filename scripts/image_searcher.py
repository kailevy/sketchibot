# -*- coding: utf-8 -*-
import urllib2
import json
import os
import cv2
import numpy as np
from PIL import Image
import io

ACCOUNT_KEY = os.environ.get('BING_API_KEY')

class ImageSearch(object):

    def __init__(self):
        credentials = (':%s' % ACCOUNT_KEY).encode('base64')[:-1]
        self.auth = 'Basic %s' % credentials
        self.root_url = "https://api.datamarket.azure.com/Bing/Search"
        self.user_agent = 'Mozilla/4.0 (compatible; MSIE 7.0; Windows NT 5.1; Trident/4.0; FDM; .NET CLR 2.0.50727; InfoPath.2; .NET CLR 1.1.4322)'

    def make_request(self, query, top=5, offset=0, bw=True, drawing=True):
        if bw or drawing:
            filters = '%27&ImageFilters=%27'
        if bw:
            filters += 'Color%3aMonochrome'
        if bw and drawing:
            filters += '%2b'
        if drawing:
            filters += 'Style%3aGraphics'
        query = urllib2.quote(query)
        url = self.root_url + '/Image?' + \
            'Query=%27' + query + filters + '%27&$top=' + str(top) + '&$skip=' + str(offset) + '&$format=json'
        request = urllib2.Request(url)
        request.add_header('Authorization', self.auth)
        request.add_header('User-Agent', self.user_agent)
        request_opener = urllib2.build_opener()
        response = request_opener.open(request)
        response_data = response.read()
        json_result = json.loads(response_data)
        result_list = json_result['d']['results']
        return [result['MediaUrl'] for result in result_list]

    def find_image(self, query, num_result=10):
        res = self.make_request(query, top=num_result)
        image = []
        for result in res:
            image.append(convert_to_opencv(get_image_from_url(result)))
        return image

def get_image_from_url(url):
    fd = urllib2.urlopen(url)
    image_file = io.BytesIO(fd.read())
    im = Image.open(image_file)
    return im

def convert_to_opencv(image):
    pil_image = image.convert('RGB')
    open_cv_image = np.array(pil_image)
    # Convert RGB to BGR
    open_cv_image = open_cv_image[:, :, ::-1].copy()
    return open_cv_image


if __name__ == '__main__':
    searcher = ImageSearch()
    # results = searcher.make_request('cow',top=2,offset=3)
    #
    # print results
    #
    # image = convert_to_opencv(get_image_from_url(results[0]))

    images = searcher.find_image('cow')
    cv2.imshow('aa', images[0])
    cv2.waitKey(0)
    cv2.imshow('aa', images[1])
    cv2.waitKey(0)
    cv2.imshow('aa', images[2])
    cv2.waitKey(0)
    cv2.imshow('aa', images[3])
    cv2.waitKey(0)
