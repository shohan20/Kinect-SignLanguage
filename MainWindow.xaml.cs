//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.DepthBasics
{
    using System;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;
    using System.Linq;
    using System.Drawing;
    using System.Windows.Controls;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        /// <summary>
        /// Map depth range to byte range
        /// </summary>
        private const int MapDepthToByte = 8000 / 256;
        
        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Reader for depth frames
        /// </summary>
        private DepthFrameReader depthFrameReader = null;
      
        private BodyFrameReader bodyFrameReader = null;
        private int bodyIndex;
        int fcolor = 1;
        int countPic = 0;
        private Body[] bodies = null;
        Boolean fpaint = true;
        private DrawingImage imageSource;
        CameraSpacePoint elbowRightPosition;
        CameraSpacePoint elbowLeftPosition;
        CameraSpacePoint HeadPosition;
        CameraSpacePoint handLeftPosition;
        ColorSpacePoint chp;
        ColorSpacePoint ceb;
        /// <summary>
        /// Description of the data contained in the depth frame
        /// </summary>
        private FrameDescription depthFrameDescription = null;
            
        /// <summary>
        /// Bitmap to display
        /// </summary>
        private WriteableBitmap depthBitmap = null;
        private WriteableBitmap colorBitmap = null;
        private System.Windows.Forms.Timer timer1;
        private int counter = 0;
        private System.Object lockThis = new System.Object();
        private WriteableBitmap signImageBitmap;
        private byte[] signPixel;
        /// <summary>
        /// Intermediate storage for frame data converted to color
        /// </summary>
        private byte[] depthPixels = null;
        private byte[] pixels = null;
        double maxDepth;
        double minDepth;
        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;
        
        private DepthSpacePoint elposition;
        private DepthSpacePoint hposition;
        private DepthSpacePoint erposition;
        private CameraSpacePoint spineMidPosition;
        private CameraSpacePoint handrfPosition;
        private CameraSpacePoint handlfPosition;
        private DepthSpacePoint spMidPosition;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            // get the kinectSensor object
            this.kinectSensor = KinectSensor.GetDefault();


            // open the reader for the depth frames
            this.depthFrameReader = this.kinectSensor.DepthFrameSource.OpenReader();

            // wire handler for frame arrival
            this.depthFrameReader.FrameArrived += this.Reader_FrameArrived;

            
            
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();
            bodyFrameReader.FrameArrived += this.BodyReader_FrameArrived;

            // get FrameDescription from DepthFrameSource
            this.depthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // allocate space to put the pixels being received and converted
            this.depthPixels = new byte[this.depthFrameDescription.Width * this.depthFrameDescription.Height];
            this.signPixel = new byte[200 * 200];
            
            // create the bitmap to display
            this.depthBitmap = new WriteableBitmap(this.depthFrameDescription.Width, this.depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Gray8, null);
            this.signImageBitmap = new WriteableBitmap(200, 200, 96.0, 96.0, PixelFormats.Gray8, null);
            
            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();
        }

        

        private void BodyReader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    this.bodies = new Body[bodyFrame.BodyCount];
                    bodyFrame.GetAndRefreshBodyData(bodies);
                    Body body = bodies.Where(b => b.IsTracked).FirstOrDefault();
                    if (body != null)
                    {
                        /*using (DrawingContext dc = this.drawingGroup.Open())
                        {*/
                            
                            Joint elbowRight = body.Joints[JointType.ElbowRight];
                            Joint elbowLeft = body.Joints[JointType.ElbowLeft];
                             elbowRightPosition = elbowRight.Position;
                             elbowLeftPosition = elbowLeft.Position;
                            Joint handRight = body.Joints[JointType.HandRight];
                            Joint handLeft = body.Joints[JointType.HandLeft];
                            Joint head = body.Joints[JointType.Head];
                            Joint spineMid = body.Joints[JointType.SpineMid];
                            Joint handrf = body.Joints[JointType.HandRight];
                        Joint handlf = body.Joints[JointType.HandLeft];
                        HeadPosition = head.Position;
                            spineMidPosition = spineMid.Position;
                            handrfPosition = handrf.Position;
                        handlfPosition = handlf.Position;
                        CameraSpacePoint handRightPosition = handRight.Position;
                        spMidPosition = kinectSensor.CoordinateMapper.MapCameraPointToDepthSpace(spineMidPosition);
                        elposition = kinectSensor.CoordinateMapper.MapCameraPointToDepthSpace(elbowLeftPosition);
                            erposition= kinectSensor.CoordinateMapper.MapCameraPointToDepthSpace(elbowLeftPosition);
                        hposition = kinectSensor.CoordinateMapper.MapCameraPointToDepthSpace(HeadPosition);
                            handLeftPosition = handLeft.Position;

                       // double handd = Math.Min(elbowLeftPosition.Z, elbowRightPosition.Z);
                            maxDepth = spineMidPosition.Z * 1000.0-200  ;
                            minDepth = spineMidPosition.Z * 1000.0 - 600;
                        //}
                    }

                        }
            }
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.depthBitmap;
            }
        }

        public ImageSource ImageSource1
        {
            get
            {
                return this.signImageBitmap;
            }
        }
        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.depthFrameReader != null)
            {
                // DepthFrameReader is IDisposable
                this.depthFrameReader.Dispose();
                this.depthFrameReader = null;
            }

         

            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        /// <summary>
        /// Handles the user clicking on the screenshot button
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        

        private void ScreenshotButton_Click(object sender, RoutedEventArgs e)
        {
            if (counter != 0)
                return;
            if (roll.Text == "" || batch.Text == "" || sign.Text == "")
                return;
            counter = 20;
            countPic = 0;
            timer1 = new System.Windows.Forms.Timer();
            timer1.Tick += new EventHandler(timer1_Tick);
            timer1.Interval = 100; // 1 second
            timer1.Start();
            text.Text = counter.ToString();
        }

        private  void timer1_Tick(object sender, EventArgs e)
        {
            lock (lockThis)
            {
                if (counter % 2 == 0)
                {
   
                    btnStart_Click_1();
                }
                counter--;
                if (counter == 0)
                {

                    timer1.Stop();
                }
                text.Text = counter.ToString();
            }
        }

        private void btnStart_Click_1()
        {
            if (this.signImageBitmap != null)
            {
                // create a png bitmap encoder which knows how to save a .png file
                BitmapEncoder encoder = new PngBitmapEncoder();
                  
                 // create frame from the writable bitmap and add to encoder
                 
                    using (depthBitmap.GetBitmapContext())
                    {

          
                        encoder.Frames.Add(BitmapFrame.Create(signImageBitmap));

                    // string time = System.DateTime.UtcNow.ToString("hh'-'mm'-'ss'", CultureInfo.CurrentUICulture.DateTimeFormat);
                   
                        string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.Desktop);
                    myPhotos += (@"\thesiss\Training" +@"\s-"+sign.Text+@"r-"+roll.Text+@"b-"+batch.Text);
                    if (!Directory.Exists(myPhotos))  // if it doesn't exist, create
                        Directory.CreateDirectory(myPhotos);
                    string path = System.IO.Path.Combine(myPhotos, countPic + ".png");
                    countPic++;
                        // write the new file to disk
                        try
                        {
                            // FileStream is IDisposable
                            using (FileStream fs = new FileStream(path, FileMode.Create))
                            {
                            
                                encoder.Save(fs);
                            }

                            this.StatusText = string.Format(CultureInfo.CurrentCulture, Properties.Resources.SavedScreenshotStatusTextFormat, path);
                        }
                        catch (IOException)
                        {
                            this.StatusText = string.Format(CultureInfo.CurrentCulture, Properties.Resources.FailedScreenshotStatusTextFormat, path);
                        }
                    }
                }
            
        }

        /// <summary>
        /// Handles the depth frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, DepthFrameArrivedEventArgs e)
        {
            bool depthFrameProcessed = false;

            using (DepthFrame depthFrame = e.FrameReference.AcquireFrame())
            {
                if (depthFrame != null)
                {
                    // the fastest way to process the body index data is to directly access 
                    // the underlying buffer
                    using (Microsoft.Kinect.KinectBuffer depthBuffer = depthFrame.LockImageBuffer())
                    {
                        // verify data and write the color data to the display bitmap
                        if (((this.depthFrameDescription.Width * this.depthFrameDescription.Height) == (depthBuffer.Size / this.depthFrameDescription.BytesPerPixel)) &&
                            (this.depthFrameDescription.Width == this.depthBitmap.PixelWidth) && (this.depthFrameDescription.Height == this.depthBitmap.PixelHeight))
                        {
                            // Note: In order to see the full range of depth (including the less reliable far field depth)
                            // we are setting maxDepth to the extreme potential depth threshold
                            //ushort maxDepth = ushort.MaxValue;

                            // If you wish to filter by reliable depth distance, uncomment the following line:
                            //maxDepth = depthFrame.DepthMaxReliableDistance;
                            if(maxDepth<=0)
                                maxDepth = 1300;
                            if(minDepth <= 0)
                                minDepth = 1000;

                            this.ProcessDepthFrameData(depthBuffer.UnderlyingBuffer, depthBuffer.Size);
                            depthFrameProcessed = true;
                        }
                    }
                }
            }

            if (depthFrameProcessed)
            {
                this.RenderDepthPixels();
            }
        }

        /// <summary>
        /// Directly accesses the underlying image buffer of the DepthFrame to 
        /// create a displayable bitmap.
        /// This function requires the /unsafe compiler option as we make use of direct
        /// access to the native memory pointed to by the depthFrameData pointer.
        /// </summary>
        /// <param name="depthFrameData">Pointer to the DepthFrame image data</param>
        /// <param name="depthFrameDataSize">Size of the DepthFrame image data</param>
        /// <param name="minDepth">The minimum reliable depth value for the frame</param>
        /// <param name="maxDepth">The maximum reliable depth value for the frame</param>
        

        private unsafe void ProcessDepthFrameData(IntPtr depthFrameData, uint depthFrameDataSize)
        {
            // depth frame data is a 16 bit value
            ushort* frameData = (ushort*)depthFrameData;
            
            // convert depth to a visual representation
            for (int i = 0; i < (int)(depthFrameDataSize / this.depthFrameDescription.BytesPerPixel); ++i)
            {
                // Get the depth for this pixel
                var depth = frameData[i];
                var y = i / (int)this.depthFrameDescription.Width;
               
                //var cmin = Length(x, y, minDepth);
                //var cmax = Length(x, y, maxDepth);
                // To convert to a byte, we're mapping the depth value to the byte range.
                // Values outside the reliable depth range are mapped to 0 (black).
                this.depthPixels[i] = (byte)(depth >= minDepth && depth <= maxDepth ? 255 : 0);
               
               
                if (y >= spMidPosition.Y)
                    this.depthPixels[i] = 0;
            }
        }

        /// <summary>
        /// Renders color pixels into the writeableBitmap.
        /// </summary>
        private void RenderDepthPixels()
        {

            this.depthBitmap.WritePixels(
                new Int32Rect(0, 0, this.depthBitmap.PixelWidth, this.depthBitmap.PixelHeight),
                this.depthPixels,
                this.depthBitmap.PixelWidth,
                0);
            if(Convert.ToInt32(elposition.X)>=50 && (Convert.ToInt32(elposition.X)+200)< (this.depthBitmap.PixelWidth+50) && (Convert.ToInt32(hposition.Y)+200)< this.depthBitmap.PixelHeight+100 && Convert.ToInt32(hposition.Y)>=100)
                this.depthBitmap.CopyPixels(new Int32Rect(Convert.ToInt32(elposition.X-50), Convert.ToInt32(hposition.Y-100), 200, 200), this.signPixel, 200, 0);
          
            this.signImageBitmap.WritePixels(new Int32Rect(0, 0, 200, 200), this.signPixel, 200, 0);

        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }
    }
}
