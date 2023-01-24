# **********************************************************************************************************************
#   FileName:
#       plotter.py
#
#   Description:
#       Utilities for plotting data from a CAN bus connection to OrbitESC
#
#   07/24/2022 | Brandon Braun | brandonbraun653@gmail.com
# **********************************************************************************************************************

import time
import can
import matplotlib.pyplot as plt
import numpy as np
from threading import Event
from collections import deque
from pyorbit.observer import MessageObserver
from pyorbit.can_messages import BaseMessage


class BlitManager:
    """ https://matplotlib.org/stable/tutorials/advanced/blitting.html """

    def __init__(self, canvas, animated_artists=()):
        """
        Parameters
        ----------
        canvas : FigureCanvasAgg
            The canvas to work with, this only works for subclasses of the Agg
            canvas which have the `~FigureCanvasAgg.copy_from_bbox` and
            `~FigureCanvasAgg.restore_region` methods.

        animated_artists : Iterable[Artist]
            List of the artists to manage
        """
        self.canvas = canvas
        self._bg = None
        self._artists = []

        for a in animated_artists:
            self.add_artist(a)
        # grab the background on every draw
        self.cid = canvas.mpl_connect("draw_event", self.on_draw)

    def on_draw(self, event):
        """Callback to register with 'draw_event'."""
        cv = self.canvas
        if event is not None:
            if event.canvas != cv:
                raise RuntimeError
        self._bg = cv.copy_from_bbox(cv.figure.bbox)
        self._draw_animated()

    def add_artist(self, art):
        """
        Add an artist to be managed.

        Parameters
        ----------
        art : Artist

            The artist to be added.  Will be set to 'animated' (just
            to be safe).  *art* must be in the figure associated with
            the canvas this class is managing.

        """
        if art.figure != self.canvas.figure:
            raise RuntimeError
        art.set_animated(True)
        self._artists.append(art)

    def _draw_animated(self):
        """Draw all the animated artists."""
        fig = self.canvas.figure
        for a in self._artists:
            fig.draw_artist(a)

    def update(self):
        """Update the screen with animated artists."""
        cv = self.canvas
        fig = cv.figure
        # paranoia in case we missed the draw event,
        if self._bg is None:
            self.on_draw(None)
        else:
            # restore the background
            cv.restore_region(self._bg)
            # draw all the animated artists
            self._draw_animated()
            # update the GUI state
            cv.blit(fig.bbox)
        # let the GUI event loop process anything it has to do
        cv.flush_events()


class LivePlotter:
    """ """

    TIME_AXIS = 0
    DATA_AXIS = 1

    def __init__(self, message: BaseMessage, attr_key: str, time_key: str = None, sample_history: int = 50):
        """
        Args:
            message: Prototype message to use for plotting
            attr_key: Which attribute to plot from the message
            time_key: If the message contains some kind of time information, what key is it under?
            sample_history: How much data to store
        """
        self._y_axis = deque(np.zeros(sample_history))
        self._x_axis = deque(np.zeros(sample_history))
        self._observer = MessageObserver(func=self.can_observer, msg_type=message)
        self._message = message
        self._attr_key = attr_key
        self._time_key = time_key
        self._plot_signal = Event()
        self._start_time = time.time()

        # Initial configuration of the figure
        self._figure, self._ax = plt.subplots()
        # (self._line,) = self._ax.plot(self._y_axis, animated=True)
        (self._line,) = self._ax.plot(self._x_axis, self._y_axis, animated=True)
        self._figure.canvas.draw()

        # Attach the figure to the BlitManager
        self._blit_manager = BlitManager(self._figure.canvas, [self._line])

    @property
    def observer_handle(self) -> MessageObserver:
        return self._observer

    def live_animate(self):
        while True:
            # Wait to receive the update signal
            self._plot_signal.wait(1.0)
            self._plot_signal.clear()

            self._line.set_ydata(self._y_axis)
            self._line.set_xdata(self._x_axis)

            # Auto-range the graph and set the data values
            self._ax.set_ylim(ymin=min(self._y_axis) * 0.90, ymax=max(self._y_axis) * 1.1)
            self._ax.set_xlim(xmin=min(self._x_axis), xmax=max(self._x_axis))

            # Redraw!
            self._blit_manager.update()

    def can_observer(self, can_msg: can.Message) -> None:
        """
        Observer to receive some data from the CAN bus
        Args:
            can_msg: Message that was received

        Returns:
            None
        """
        assert isinstance(can_msg, self._message.__class__)
        self._message = can_msg

        # Get the timestamp used for plotting
        time_data = time.time() - self._start_time
        if self._time_key:
            time_data = self._message.get_keyed_data(self._time_key)

        # Get the real data being plotted
        real_data = self._message.get_keyed_data(self._attr_key)

        # Insert the data into the buffer, then trigger redraw
        self._y_axis.popleft()
        self._x_axis.popleft()
        self._y_axis.append(real_data)
        self._x_axis.append(time_data)
        self._plot_signal.set()
