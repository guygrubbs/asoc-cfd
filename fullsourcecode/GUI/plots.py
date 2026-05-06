import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import time
import pyqtgraph as pg
from PyQt5.QtGui import QTransform


# This class creates and updates the hitmap on the main window of the GUI
class HeatmapWidget(QtWidgets.QWidget):
    # init function creates the plot
    def __init__(self, parent=None):
        super().__init__(parent)

        layout = QtWidgets.QVBoxLayout(self)

        # Graphics view
        self.view = pg.GraphicsLayoutWidget()
        layout.addWidget(self.view)

        # Plot
        self.plot = self.view.addPlot()
        self.plot.setLabel('left', 'Y-Position')
        self.plot.setLabel('bottom', 'X-Position')
        self.plot.setTitle('X-Y Hitmap')
        # self.plot.setAspectLocked(True, ratio = 1)

        # Image item
        self.img = pg.ImageItem()
        self.plot.addItem(self.img)

        # Colormap
        self.cmap = pg.colormap.get("magma")
        self.img.setColorMap(self.cmap)

        # Colorbar
        self.cbar = pg.ColorBarItem(colorMap=self.cmap)
        self.cbar.setImageItem(self.img)
        self.view.addItem(self.cbar)

        # Track current downsample factor
        self.factor = 1

        self.img.setLevels((0, 10))

    # function takes in an image with a new batch of data and adds it to the current hitmap image on the screen
    def set_image(self, image, x, y, factor=1):
        
        self.factor = factor
        sx = x / image.shape[1]
        sy = y / image.shape[0]
        
        # Update image
        self.img.setImage(image.T, autoLevels=True)

        t = QTransform()
        t.scale(sx, sy)
        t.translate(-image.shape[1] / 2, -image.shape[0] / 2)
        self.img.setTransform(t)

    # makes the hitmap update the scale automatically
    def auto_levels(self):
        self.img.setImage(self.img.image, autoLevels=True)

    # function sets the levels of the hitmap
    def set_levels(self, vmin, vmax):
        self.img.setLevels((vmin, vmax))

    # function clears the graph
    def clear(self):
        if self.img.image is not None:
            self.img.setImage(np.zeros_like(self.img.image), autoLevels=False)

# this class creates and updates a pulse height distribution plot on the main tab of the GUI
class PHDWidget(QtWidgets.QWidget):
    # function sets up the PHD with the specified number of bins
    def __init__(self, bins = 256, parent = None):
        super().__init__(parent)
        self.bins = bins
        layout = QtWidgets.QVBoxLayout(self)
        self.view = pg.GraphicsLayoutWidget()
        layout.addWidget(self.view)


        self.plot = self.view.addPlot()
        self.plot.setLabel('left', 'Counts')
        self.plot.setLabel('bottom', 'Bin')
        self.plot.showGrid(x = False, y = False)
        self.plot.setTitle('Pulse Height Distribution Histogram')

        self.bar = pg.BarGraphItem(x = np.arange(self.bins), height = np.zeros(self.bins), width = 1)

        self.plot.addItem(self.bar)
        self.hist = np.zeros(self.bins, dtype = np.int64)
        self.plot.setLimits(yMin = 0, xMin = -10, xMax = 266)
        self.plot.setXRange(0, self.bins)
        self.plot.enableAutoRange(axis = 'y', enable = False)
        self.ymax = 50
        self.plot.setYRange(0, self.ymax)
        vb = self.plot.getViewBox()
        vb.setMenuEnabled(False)
    
    # starts the plot
    def start(self):
        vb = self.plot.getViewBox()
        vb.setMenuEnabled(False)
        vb.setMouseEnabled(x = False, y = False)
    
    # stops the plot
    def stop(self):
        vb = self.plot.getViewBox()
        vb.setMenuEnabled(True)
        vb.setMouseEnabled(x = True, y = True)

    # updates the plot with the new batch of bin distributions received
    def updatePlot(self, batch):
        self.hist += batch
        self.bar.setOpts(height = self.hist)

        ymax = np.max(self.hist)
        if ymax > self.ymax:
            self.ymax = ymax
            self.plot.setYRange(0, int(ymax * 1.1))

    # clears the plot
    def clear(self):
        self.hist[:] = 0
        self.ymax = 25
        self.plot.setYRange(0, int(self.ymax * 1.1))
        self.bar.setOpts(height = self.hist)

# this class creates and plots the running event rate on the main window of the GUI
class EventRateWidget(QtWidgets.QWidget):
    # set up the plot
    def __init__(self, window = 10, parent = None):
        super().__init__(parent)

        self.window = window

        layout = QtWidgets.QVBoxLayout(self)

        # Graphics view
        self.view = pg.GraphicsLayoutWidget()
        layout.addWidget(self.view)

        # Plot
        self.plot = self.view.addPlot()
        self.plot.setLabel('left', 'Events')
        self.plot.setLabel('bottom', 'Time (s)')
        self.plot.showGrid(x = False, y = False)

        self.plot.setTitle('Running Event Rate')
        self.curve = self.plot.plot(pen = pg.mkPen(width=2))

        self.plot.enableAutoRange(axis = 'y', enable = False)
        self.plot.setLimits(yMin = 0)
        self.plot.getViewBox().setYRange(0, 1)
        
        # Data storage
        self.times = []
        self.rates = []

        self.count = 0

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.running = False
    

    # starts the plot
    def start(self):
        self.running = True
        self.clear()
        self.startTime = time.time()
        self.timer.start(1000)
        vb = self.plot.getViewBox()
        vb.setMenuEnabled(False)
        vb.setMouseEnabled(x = False, y = False)

    # clears the display
    def clear(self):
        self.count = 0
        self.times = []
        self.rates = []
        self.plot.getViewBox().setYRange(0, 1)
        self.plot.getViewBox().setXRange(0, self.window)

    # stops the plot
    def stop(self, stopAcq):
        if stopAcq:
            self.timer.stop()
        vb = self.plot.getViewBox()
        vb.setMenuEnabled(True)
        vb.setMouseEnabled(x = True, y = True)
        self.curve.setData(self.times, self.rates)
        self.running = stopAcq
    
    # updates the plot every second
    def update(self):
        now = time.time() - self.startTime
        rate = self.count
        self.count = 0
        self.times.append(now)
        self.rates.append(rate)

        if len(self.times) > self.window:
            times = self.times[-self.window:]
            rates = self.rates[-self.window:]
        else:
            times = self.times
            rates = self.rates
            
        if self.running:
            vb = self.plot.getViewBox()
            vb.setMenuEnabled(False)
            vb.setMouseEnabled(x = False, y = False)
            self.curve.setData(times, rates)

            ymax = max(rates) if rates else 1
            ymin = min(rates) if rates else 0
            self.plot.getViewBox().setYRange(ymin // 1.1, ymax * 1.1, padding = 0)
            self.plot.getViewBox().setXRange(times[0], times[-1] if len(times) == self.window else self.window)

    # adds events to the total number of processed events
    def addEvents(self, counts):
        self.count += counts
