# Sketchibot

##### Alex Crease, Jay Woo, Kai Levy

### Computational Robotics Fall 2015 Final Project: Seeing and Drawing Robot

## Description
Sketchibot is a seeing and drawing robot. It attempts to play a 'pictionary'-like game, where the user writes down a prompt for it. It reads the prompt and then proceeds to draw its interpretation of the prompt, using the Bing Search API, OpenCV's canny edge detection & contour finding, and various filtering methods.

For more information about our project and our process, see the project stories included in `/stories`

## Prerequisites
- Marker attachment on the Neato
- Installation on the raspberry pi of the scripts included in `bot_files`
  - Modification of `/etc/rc.local` to run the scripts at startup:

    `echo "sudo ~pi/sketchibot/servo_server.py &" >> /etc/rc.local`

- Addition of the following code to `bringup_minimal.launch` in `neato_node`
```
<node name="marker_node" pkg="sketchibot" type="servo_client.py" output="screen">
  <param name="host" value="$(arg host)" />
</node>
```
- [Bing API Key](http://www.bing.com/toolbox/bingsearchapi), stored as an the environmental variable `BING_API_KEY`
- [OpenCV](https://help.ubuntu.com/community/OpenCV)
- Numpy and its dependencies
  - `apt-get install python-numpy`
- Tesseract OCR and pytesseract
  - `apt-get install tesseract-ocr`
  - `pip install pytesseract`
- Hunspell
  - `apt-get install libhunspell-dev`
  - `pip install hunspell`

## Usage
- Connect to the Neato
  - `roslaunch neato_node bringup.launch host:=[ip of robot]`
- Launch the script
  - `roslaunch sketchibot sketchibot.launch`

## System Architecture
Sketchibot's work flow requires the coordination of a number of different modular parts described below.

### Text Reading

### Image Searching

### Edge Detection

### Contour Filtering

### Navigation
