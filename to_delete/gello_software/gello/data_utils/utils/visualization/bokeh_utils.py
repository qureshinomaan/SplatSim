from bokeh.plotting import figure, output_file, show
from bokeh.io import export_png, export_svg
from bokeh.models import Range1d
import numpy as np


def bh_scatter_points(
    points, colors, file_name=None, *, marker="circle", size=7, fill_alpha=0.8, x_lim=None, y_lim=None, p=None, resolution=(512, 512)
):
    if p is None:
        p = figure(plot_width=resolution[0], plot_height=resolution[1], output_backend="webgl")
        p.background_fill_color = None
        p.border_fill_color = None
        p.toolbar.logo = None
        p.toolbar_location = None

    if x_lim is not None:
        p.x_range = Range1d(x_lim[0], x_lim[1])
    if y_lim is not None:
        p.y_range = Range1d(y_lim[0], y_lim[1])

    extra_args = {}
    if size is not None:
        extra_args["size"] = size
    p.scatter(points[:, 0], points[:, 1], marker=marker, fill_color=colors, fill_alpha=fill_alpha, line_color=None, **extra_args)
    if file_name is None:
        pass
    elif file_name.endswith("html"):
        output_file("file_name", title="Scatter points")
        show(p)
    elif file_name.endswith("png"):
        export_png(p, filename=file_name)
    else:
        raise NotImplemented("")
    return p


if __name__ == "__main__":
    pass
