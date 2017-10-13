# End-to-End-Learning-for-Self-Driving-Cars

## Introduction
This is the third project of the Udactiy's Self-Driving Car Nanodegree. In this project, we trained a Convulutional Nerual Network model to learn to adjust the steering wheels to keep the car on the road. During training, the input to the model is the images aquired from three cameras behind the windshield of the simulated car. However during the test, we only rely on the images acquited from the central camera to steer the simulated car. The final test videos on the two track are here.</br>
[![IMAGE ALT TEXT](http://img.youtube.com/vi/Epqb7tTIe6s/0.jpg)](https://youtu.be/Epqb7tTIe6s "a small networkd ")
[![IMAGE ALT TEXT](http://img.youtube.com/vi/gnh3lAgt-MU/0.jpg)](https://youtu.be/gnh3lAgt-MU "a small networkd ")



## Software
#### Software requirements:
* numpy, flask-socketio, eventlet, pillow, h5py, keras, Tensorflow</br>
* Simulator can be found at:</br>
[Windows 64 bit](https://d17h27t6h515a5.cloudfront.net/topher/2016/November/5831f3a4_simulator-windows-64/simulator-windows-64.zip)</br>
[Windows 32 bit](https://d17h27t6h515a5.cloudfront.net/topher/2016/November/5831f4b6_simulator-windows-32/simulator-windows-32.zip)</br>
[macOS](https://d17h27t6h515a5.cloudfront.net/topher/2016/November/5831f290_simulator-macos/simulator-macos.zip)</br>
[Linux](https://d17h27t6h515a5.cloudfront.net/topher/2016/November/5831f0f7_simulator-linux/simulator-linux.zip)</br>

#### Steps to train and run the model
1. Download the data [here](https://d17h27t6h515a5.cloudfront.net/topher/2016/December/584f6edd_data/data.zip)
2. Train and run the model 

```
python model.py

python drive.py model.json

```

Finally open the simulator and select autonomous mode</br>

#### The simulator front page
![](/images/simulator.png "simulator")

## Data
The Training data can be dolownded from [here](https://d17h27t6h515a5.cloudfront.net/topher/2016/December/584f6edd_data/data.zip). This is the training data on track one provided by Udacity. We did not collect our own data because of the difficulty of collecting data using keyborad.</br>
Sample data from cameras:</br>
![](/images/picture3.png "simulator")
Statistics of the steering angles from Udacity data:
![](/images/picture2.png "simulator")


## Augmentation/Preprocessing
From above, we can see that the training data is not banlanced, so we need augment it. I use three ways to augment/preprocess the data:</br> 

1. **Resize and change color space** -resize the input image seize to (16,32), and change to HSV color channel. In the final architecures, We only use the S channel. I also tried used all three RGB or HSV channel but the S channel only produce the best results in my case.
2. **Use Left/right camera** - Images from left/right camera are also used by modifiying the steering angle with 0.25. We should notice that, adding a constant angle to steering is a simplified version of shifting left and right cameras, but not the best way. But in our case, this simplificaiton is good enough.</br> 
3. **Flip the images** - Flip the images from all three cameras to account for the situation of driving in the opposite way.This also increase our training data.</br> 

I also shuffle and split the data in to training(90%) and validation(10%) datasets. But the best way to validate and test the resultsing model is to run in on both track.</br>

Because we resize the image size to (16,32), which largely reduced the training data size, we did not use the generator.


## Model architecture
I start with the Nvidia End-to-end learning deep learning architecture. But it turns out that it is very difficult to train Nvidia models with our training data, because our data is not big enough to fully train the Nvidia model unless very heavy augmentation techquies is implemented, such as the discription [here](https://chatbotslife.com/using-augmentation-to-mimic-human-driving-496b569760a9#.p9gqjosuv).
Untill I saw the model of Mengxi Wu, he then wrote a article introducing his [tiny model](https://medium.com/@xslittlegrass/self-driving-car-in-a-simulator-with-a-tiny-neural-network-13d33b871234#.8fj065dgy). I realized that I need a much smaller model compared Nvidia's, to better match my training data. Here is the model architecture which works in my case.</br>
</br>
0. Input data normorlization
1. Convolution layer with 3x3 kernel and 1x1 stride</br>
2. relu activation layer</br>
3. MaxPooling with 2x2 pool size
4. Convolution layer with 3x3 kernel and 1x1 stride</br>
5. relu activation layer</br>
6. MaxPooling with 2x2 pool size</br>
7. Dropout - Prevents overfitting.</br> 
8. Flatten</br>
9. Dense layer with 50 neuron</br>
10. relu activation layer</br>
11. Dense layer with 1 neuron</br> 
</br>
## Tuning and Hyperparameters
I have been trying to adopt SGD optimizer with learning rate 0.01, 0.001, 0.0005 and 0.0001,and also different batch size and training epoch. For batch size, I tried 64, 128 and 256; and training epoch ranging from 10 to 50. It is really a lot of work and you just have to try to see how the simulated car behave. 

I found that: there is really no help to train more than 10 epochs even if the validation accuracy is keeping decrease; that Adam optimizer is better because of its adaptive learning rates; batch size 128 can do a good job.

So finnay I settled with Adam optimizer with default leraning rate 0.001, batch size 128 and traing epoch 10.

