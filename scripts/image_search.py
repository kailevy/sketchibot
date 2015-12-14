# -*- coding: utf-8 -*-
"""
Accesses Bing Image API to retrieve images with a given query
Requires an API Key:
`. api_key.sh`
"""
import urllib2, json, os, io
import cv2
import numpy as np
from PIL import Image

ACCOUNT_KEY = os.environ.get('BING_API_KEY')
BW = 'Color%3aMonochrome'
DRAWING = 'Style%3aGraphics'
MEDIUM = 'Size%3aMedium'

class ImageSearcher(object):
    """
    Wrapper for Bing Image Search
    """

    def __init__(self):
        credentials = (':%s' % ACCOUNT_KEY).encode('base64')[:-1]
        self.auth = 'Basic %s' % credentials
        self.root_url = "https://api.datamarket.azure.com/Bing/Search"
        self.user_agent = 'Mozilla/4.0 (compatible; MSIE 7.0; Windows NT 5.1; Trident/4.0; FDM; .NET CLR 2.0.50727; InfoPath.2; .NET CLR 1.1.4322)'

    def make_request(self, query, top=5, skip=0, filters=[], clipart=True):
        """
        Retrieves top results, with skip, optional filters, and an optional 'clip art' addendum
        """
        filter_str = '%27&ImageFilters=%27'+'%2b'.join(filters) # Add filters
        if clipart:
            query = query + ' clip art'
        query = urllib2.quote(query)
        # Construct query url
        url = self.root_url + '/Image?' + \
            'Query=%27' + query + filter_str + '%27&$top=' + str(top) + '&$skip=' + str(skip) + '&$format=json'
        # Make request
        request = urllib2.Request(url)
        request.add_header('Authorization', self.auth)
        request.add_header('User-Agent', self.user_agent)
        request_opener = urllib2.build_opener()
        response = request_opener.open(request)
        response_data = response.read()
        json_result = json.loads(response_data)
        result_list = json_result['d']['results']
        # Return direct urls to images
        return [result['MediaUrl'] for result in result_list]

    def find_image(self, query, top=10, skip=0, filters=[], clipart=True):
        """
        Calls querying method and creates a list of images from the results
        """
        res = self.make_request(query, top=top, skip=skip, filters=filters, clipart=clipart)
        image = []
        for result in res:
            image.append(convert_to_opencv(get_image_from_url(result)))
        return image

def get_image_from_url(url):
    """
    Retrieves image from given url
    """
    fd = urllib2.urlopen(url)
    image_file = io.BytesIO(fd.read())
    im = Image.open(image_file)
    return im

def convert_to_opencv(image):
    """
    Converts PIL Image to OpenCV Image
    """
    pil_image = image.convert('RGB')
    open_cv_image = np.array(pil_image)
    # Convert RGB to BGR
    open_cv_image = open_cv_image[:, :, ::-1].copy()
    return cv2.cvtColor(open_cv_image, cv2.COLOR_BGR2GRAY)
