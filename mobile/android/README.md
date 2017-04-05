# Multi-Sensor Grabber

App that is able to grab videos (in form of image-frames) that are annotated with

* GPS
* Accelerometer
* Gyroscope
* Timestamp

to **create low-cost, mobile sequence capturing devices**.

Playstore: https://play.google.com/store/apps/details?id=de.weis.multisensor_grabber

![Multi-Sensor Grabber](app_screenshot.png "Multi-Sensor Grabber")

## Currently implemented
* Acquire rights (camera, gps, etc.)
* Fullscreen app with overlayed button start/stop capturing
* Getting and setting desired image resolution
* Selection of image resolution
* Selection of framerates (10,15,...)
* Frame includes: Timestamp, Image, GPS-data (lat,lon,accuracy,speed) 
* Save images to folder, metadata to xml-file

Below is a snippet from an example capture, names are adapted to a database system I use in the backend:

| tag | name | meaning |
| --- | --- | --- |
|sequence | folder | storage folder on phone |
| | sensor | identifier of the sensor (here: phone) used |
| | ts | timestamp when sequence has started |
| | whitebalance | value the whitebalance has been set to |
| frame |  uri | filename (relative to sequence folder) |
| | lat | Latitude |
| | lon | Longitude |
| | acc | gps accuracy as reported by locationManager |
| | img_w | width of image |
| | img_h | height of imgae |
| | speed | speed in m/s as reported by locationManager |
| | ts_cam | timestamp of image acquisition |
| | avelx | angular velocity around x-axis |
| | avely | angular velocity around y-axis |
| | avelz | angular velocity around z-axis |
| | accx | accelerometer value about x-axis |
| | accy | accelerometer value about y-axis |
| | accz | accelerometer value about z-axis |

```xml
<sequence 
    folder="/storage/0000-0000/Android/data/com.example.weis.cv_grabber/files/multisensorgrabber_1482584089332" 
    sensor="samsungSM-G903F" 
    ts="1482584089332" 
    whitebalance="auto"
>
    <Frame 
        uri="IMG_1482584089688.jpg" 
        lat="50.16629447649796" 
        lon="8.658641955498677" 
        acc="48.0" 
        img_w="960" 
        img_h="720" 
        speed="0.0" 
        ts_cam="1482584089688" 
        avelx="-0.11246156692504883" 
        avely="0.05926918238401413" 
        avelz="-0.20069235563278198" 
        accx="9.873687744140625" 
        accy="0.18809446692466736" 
        accz="1.9354127645492554"
    />
    <Frame 
        uri="IMG_1482584089809.jpg" 
        lat="50.16629447649796" 
        lon="8.658641955498677" 
        acc="48.0" 
        img_w="960" 
        img_h="720" 
        speed="0.0" 
        ts_cam="1482584089809" 
        avelx="0.06863339245319366" 
        avely="0.09868396818637848" 
        avelz="-0.20708394050598145" 
        accx="9.767146110534668" 
        accy="-0.00823006872087717" 
        accz="1.6624737977981567"
    />

    ...
</sequence>
```

## TODO
* Implement func to enable user to save all available camera settings and send to me
* Upload functionality and db-integration in backend
* ~~Better control of camera parameters~~ Now using camera2 API
* ~~Settings Activity?~~
* ~~choose framerate (and implement)~~
* ~~gps grabbing~~
* ~~accel/gyro grabbing~~
* Saving: ~~xml~~/sqlite ?
