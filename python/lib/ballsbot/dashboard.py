from copy import deepcopy
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure
from io import BytesIO
from math import sqrt, ceil

from ballsbot.utils import figsize_from_image_size
from ballsbot.config import ENABLE_MULTIPROCESSING
import ballsbot.drawing


class Dashboard:
    def __init__(self, widgets, image_size=640):
        self.image = widgets.Image(format='jpeg', width=image_size, height=image_size)
        self.figsize = figsize_from_image_size(self.image)
        self.function_by_plot = {}
        self.current_data = {}

    def get_image(self):
        return self.image

    def add_subplot(self, plot_name, function_name):
        self.function_by_plot[plot_name] = {
            'plot_name': plot_name,
            'function_name': function_name,
            'index': len(self.function_by_plot.keys()) + 1,
        }

    def set_subplot_data(self, plot_name, parameters):
        if plot_name not in self.function_by_plot:
            raise ValueError(f'unknown plot_name "{plot_name}" (only {self.function_by_plot} registered)')
        self.current_data[plot_name] = deepcopy(parameters)

    def redraw(self):
        data = self.current_data
        self.current_data = {}

        plot_size = ceil(sqrt(len(self.function_by_plot.keys())))
        if ENABLE_MULTIPROCESSING:
            drawer = DrawInAnotherProcess.get_instance('dashboard_drawing_images')
            value = drawer.draw('_get_dashboard_image', data, self.function_by_plot, plot_size, self.figsize)
        else:
            value = _get_dashboard_image(data, self.function_by_plot, plot_size, self.figsize)

        self.image.value = value


def _get_dashboard_image(data, function_by_plot, plot_size, figsize):
    fig = Figure(figsize=figsize)
    canvas = FigureCanvas(fig)

    for it in function_by_plot.values():
        ax = fig.add_subplot(plot_size, plot_size, it['index'])
        parameters = data.get(it['plot_name'])
        if parameters:
            func = getattr(ballsbot.drawing, it['function_name'])
            func(ax, *parameters)

    fig.tight_layout()
    canvas.draw()
    jpeg = BytesIO()
    canvas.print_jpg(jpeg)
    return jpeg.getvalue()


if ENABLE_MULTIPROCESSING:
    import multiprocessing as mp

    if not mp.get_start_method(allow_none=True):
        mp.set_start_method('spawn')


class DrawInAnotherProcess:
    @classmethod
    def get_instance(cls, instance_name):
        instances = getattr(cls, 'instances', None)
        if instances is None:
            instances = {}
            setattr(cls, 'instances', instances)
        if instance_name in instances:
            instance = instances[instance_name]
        else:
            instance = cls(instance_name)
            instances[instance_name] = instance
        return instance

    @classmethod
    def stop_all(cls):
        instances = getattr(cls, 'instances', None)
        if instances is None:
            return
        for instance in instances.values():
            instance.stop()
        instances.clear()

    def __init__(self, name):
        self.conn, child_conn = mp.Pipe()
        self.process = mp.Process(
            name=f'DrawInAnotherProcess:{name}',
            target=self._processing_cycle,
            args=(child_conn,)
        )
        self.process.start()

    def draw(self, function_name, *params):
        self.conn.send([function_name, params])
        return self.conn.recv()

    def stop(self):
        self.conn.send(['stop', []])
        self.process.join()

    @staticmethod
    def _processing_cycle(conn):
        while True:
            function_name, params = conn.recv()
            if function_name == 'stop':
                conn.close()
                break
            result = globals()[function_name](*params)
            conn.send(result)
