# Learning objects

## Capturing images

Start the robot

    astart
    amiddle

Position the robot in front of the objects, and let the head look at the objects. Open a robot console (e.g. for AMIGO):

    amigo-console

Now you can capture an image and save it to disk using:

    amigo.ed.save_image(path="/some/path")

This will store the RGBD-image (color + depth) and meta-data (such as the timestamp and 6D pose) to the path you specified. If this path does not yet exist, it will be created. The filename of the file will be the date and timestamp of the image captured.

## Annotating the images

**Before you can annotate the images, you need a 3D (ED) model of the supporting furniture (e.g. the table on which the objects are positioned)**. Let's say the name of this model is `my-lab/table`. Start the annotator with the path in which you stored the images:

    rosrun ed_perception annotation-gui /some/path

No you can cycle through the images using the arrow keys. Note that the segmentation is still pretty bad. This will however change once you add the supporting furniture.

  * In the annotation-gui, type the name of the supporting entity, in our case `my-lab/table`. You should see the entity name appearing.
  * Once you typed the name, press enter. The color of the name should change to red. This means that it is selected
  * Left-click in the image on the location where the supporting (the table) is. A label should appear
  * If you made a mistake, right-click on the blue sphere to delete it.

Go through all the images and annotate the supporting entity.

Once an image has the supporting entity annotated, segmentation should improve. Now you can annotate the rest objects in the images, using the same process:

  * Type the name of the object an press enter
    * **If you already typed the name, you can use auto-completion! Use the up and down arrows to select the object**
  * Left-click on the (middle of the) object in the image
  * *Tip: type the name of the object, then go through all the images and click on it. In general this is faster then re-typing the name of the object, even with auto-completion*

You can always exit the annotation-gui by pressing ESC. Your progress will be saved (in the json-meta-data files).

## Training the perception modules

Go to the package in which the perception modules are stored:

    roscd ed_perception_models
    cd models

Create a new folder with the name of the current environment:

   mkdir $ROBOT_ENV

Within this directory, create a file called `parameters.yaml` with a content like this:

    modules:
    - lib: libcolor_matcher.so
      classification:
        color_margin: 0.02
    - lib: libsize_matcher.so
      classification:
        size_margin: 0.01

This determines which perception modules are used, with which parameters. Now you can train the models for these perception modules, using the `train-perception` tool. This takes two arguments:

    rosrun ed_perception train-perception <config-file> <image-file-or-directory>

So in our case:

    rosrun ed_perception train-perception `rospack find ed_perception_models`/models/$ROBOT_ENV/parameters.yaml /path/to/images

## Test the perception models  

    rosrun ed_perception test-perception `rospack find ed_perception_models`/models/$ROBOT_ENV/parameters.yaml /path/to/images

