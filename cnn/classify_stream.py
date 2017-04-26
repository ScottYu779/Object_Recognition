import numpy as np
import os
import matplotlib.pyplot as plt
from PIL import Image
import caffe
import cv2
import time

def classify(img):
	
	net.blobs['data'].data[...] = transformer.preprocess('data', img)
	
	output = net.forward()
	
	if max(output['loss'][0])>classification_threshold:
		
		return output['loss'].argmax()
			 
	else:
		return -1
	

caffe.set_device(0)
caffe.set_mode_gpu()

model = 'ardopnet_deploy.prototxt';
weights = 'ardopnet_iter_237.caffemodel';


net = caffe.Net(model,weights,caffe.TEST)

classification_threshold = 0.5

transformer = caffe.io.Transformer({'data': net.blobs['data'].data.shape})
transformer.set_raw_scale('data', 255.0)
net.blobs['data'].reshape(1,1,32,32)



#fil = os.listdir('/home/ubuntu/ardop/stereo_app/objects')
#while(1):
#time.sleep(0.5)


fil1 = open("/home/ubuntu/ardop/stereo_app/mutex.txt",'r+')
fil = open("/home/ubuntu/ardop/stereo_app/coordinates.txt",'r')


lines = fil.readlines()
fil.close()
m = fil1.readline()
fil1.close()
while(1):
	time.sleep(0.1)
	classification_list = []
	if(int(m,10)==0):
		for f in lines:
			fs = f.split()
			
			img = caffe.io.load_image('/home/ubuntu/ardop/stereo_app/objects/'+fs[0]+'.jpg',0)
			img = cv2.resize(img, (32, 32), interpolation = cv2.INTER_CUBIC)
			#img = cv2.equalizeHist(img);
			c = classify(img)
			#print str(c)
			classification_list.append(str(c))
		
		fil2 = open("/home/ubuntu/ardop/stereo_app/c.txt",'wb')
		#fil2.seek(0)
		fil2.truncate()
		fil2.write("\n".join(classification_list))
		print ' '.join(classification_list)
		
		fil2.close()
		
		fil1 = open("/home/ubuntu/ardop/stereo_app/mutex.txt",'r+')
		fil1.seek(0)
		fil1.write('1')
		fil1.close()
		
	
	else:
		print "Waiting for mutex"
		#fil1.close()
		#fil2.close()
		
		
	
	

