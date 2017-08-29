#Overview
This repository contains code that allows you to train and classify objects in real time. The code was written to run on an NVIDIA Jetson TK1, but should run on any linux system.
The classified objects were localized using a stereo camera (in this case a DUO MLX stereo camera). If another camera is being used, the input stream needs to be modified in the `./Stereo/Sample.cpp` file and rebuilding the program by running 'make'.
At this phase of the project, we have restricted the system to only classify objects placed on a table with a uniform background and the ROI is limited the table. 
As Convolution Neural Networks (CNN’s) are used for object classification, the exact regions of the frame from the camera stream that contain the objects must be obtained. A contour based localization system is used for this purpose. Bounding boxes are constructed over the contour regions that correspond to objects in the scene and the images obtained from these boxes are fed to the CNN for classification. After this procedure, the identities of the objects as well as its position with respect to the image are known. The position of each of the objects are then determined by the stereo camera.

#Prerequisites
1.Install caffe by following the instructions at (http://caffe.berkeleyvision.org/installation.html). 
2. Clone the repository using `git clone https://github.com/ardop/Object_Recognition.git`
3. If you're using the DUO MLX stereo camera, follow the instructions and the documentaton on the website (https://duo3d.com/).

#Data Preparation
The training data is created by running: 'sudo ./data_collection/photo`
Place the object on the table and press the **space bar** to take images. You will have to make sure that your object is the only one being detected in the scene.  
This process is to be repeated for various positions and orientations and after taking sufficient images, close the program and move the images into a separate folder. This process needs to be repeated for all objects that you wish to train.  
If you wish to restart the program but take more images of the same class later, modify **line 107** in `./data_collection/Photo/Sample.cpp` to continue saving images from that index. Reset this back to 1 once you've finished. 
You will have to rebuild the program everytime you modify the code by running `make` in `./data_collection'. 

#Training 
1. In order to train the model to fit your training data, modify **lines 15-18 and lines 34-37** in `./cnn/ardopnet_train_test.prototxt` to specify the root directory of your data and the batchsize you wish to use.
2. If you have more than 2 classes of objects, modify **line 373** to the number of classes that you have. You will also have to modify **line 341** in './cnn/ardopnet_deploy.prototxt`.
3. Specify the paths to all your files used for training and test in `./cnn/train.txt` and `./cnn/test.txt` along with the class number like in the sample.
4. You will have to play around with the values of learning rate, number of iterations, etc in `./cnn/ardopnet_solver.prototxt` for results specific to your data set. 
5. Run the training process using `sudo sh /cnn/train_ardopnet.sh`. When the training is finished or interupted, the weights will be saved as `ardopnet_iter_X.caffemodel`.

#Implementation
1. Modify **line 27** in `./cnn/classify_stream.py` to the name of the file with the trained weights, `ardopnet_iter_X.caffemodel`.
2. Open two terminals. In one terminal run `sudo ./stereo` and in another run `python classify_stream.py'.
The first program will find the bounding boxes of the objects and save the images in a folder. The second program will input the saved images and classify them.
The position of each of the objects are determined by the first program using the stereo camera and the file required for the inverse kinematic model is generated, `./ik_file.txt`.
The process is synchronized using `mutex.txt` and the communication between the program occurs through `coordinates.txt and c.txt`. 
