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
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from threading import Event
from collections import deque
from pyorbit.pipe import MessageObserver
from pyorbit.messages import PowerSupplyVoltage
from loguru import logger


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

    def __init__(self, arb_id: int, sample_history: int = 50):
        self._accumulator = deque(np.zeros(sample_history))
        self._observer = MessageObserver(func=self.can_observer, arbitration_id=arb_id, persistent=True)

        self._figure, self._ax = plt.subplots()
        (self._line,) = self._ax.plot(self._accumulator, animated=True)

        self._figure.canvas.draw()
        self._blit_manager = BlitManager(self._figure.canvas, [self._line])
        self._ax.set_ylim([0.0, 10.0])

        self._plot_signal = Event()

    @property
    def observer_handle(self) -> MessageObserver:
        return self._observer

    def redraw_from_main(self):
        while True:
            self._plot_signal.wait(1.0)
            self._plot_signal.clear()
            self._line.set_ydata(self._accumulator)
            self._blit_manager.update()
            logger.debug("Plot new data")

    def can_observer(self, can_msg: can.Message, timeout: bool) -> None:
        # Fill in the data
        msg = PowerSupplyVoltage().unpack(bytes(can_msg.data))
        voltage = msg.vdd/1e6

        self._accumulator.popleft()
        self._accumulator.append(voltage)
        self._plot_signal.set()

