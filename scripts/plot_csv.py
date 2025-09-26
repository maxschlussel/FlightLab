import math
import pandas as pd
import plotly.graph_objects as go
import numpy as np

from plotly.subplots import make_subplots
import plotly.offline as pyo


def plot_csv(csv_file: str = 'output/data_log.csv', nrows: int = 3, ncols: int = 3):
    """
    Generates an interactive, multi-page plot of flight data from a CSV file.
    """

    df = pd.read_csv(csv_file, skipinitialspace=True)
    
    # df.to_numpy()
    # df = df[:800].astype(float)

    df = df.replace('-nan(ind)', np.nan).astype(float)
    
    df["u_cmd_da"] *= 180/math.pi
    df["u_cmd_de"] *= 180/math.pi
    df["u_cmd_dr"] *= 180/math.pi
    df["srvo_aileron_pos"] *= 180/math.pi
    df["srvo_elevator_pos"] *= 180/math.pi
    df["srvo_rudder_pos"] *= 180/math.pi
    df["alpha"] *= 180/math.pi
    df["beta"] *= 180/math.pi
    df["Xdot_p"] *= 180/math.pi
    df["Xdot_q"] *= 180/math.pi
    df["Xdot_r"] *= 180/math.pi
    df["Xdot_phi"] *= 180/math.pi
    df["Xdot_theta"] *= 180/math.pi
    df["Xdot_psi"] *= 180/math.pi
    df["X_p"] *= 180/math.pi
    df["X_q"] *= 180/math.pi
    df["X_r"] *= 180/math.pi
    df["X_phi"] *= 180/math.pi
    df["X_theta"] *= 180/math.pi
    df["X_psi"] *= 180/math.pi
    df["u_cmd_dthr1"] *= 180/math.pi
    df["u_cmd_dthr2"] *= 180/math.pi

    time = df.iloc[:, 0]
    headers = list(df.columns[1:])  # everything except time
    per_page = nrows * ncols
    total = len(headers)
    pages = math.ceil(total / per_page)

    # Create subplot grid
    # tweak horizontal/vertical spacing to give room for titles/buttons
    fig = make_subplots(rows=nrows, cols=ncols,
                        shared_xaxes=False, shared_yaxes=False,
                        horizontal_spacing=0.06, vertical_spacing=0.08)

    # Add traces and record metadata
    trace_meta = []  # (page, slot, row, col, name)
    for i, name in enumerate(headers):
        page = i // per_page
        slot = i % per_page
        r = slot // ncols + 1
        c = slot % ncols + 1
        trace = go.Scatter(
            x=time, y=df[name],
            mode="lines", name=name,
            visible=(page == 0)
        )
        fig.add_trace(trace, row=r, col=c)
        trace_meta.append((page, slot, r, c, name))

    # Add gridlines and independent axes
    for r in range(1, nrows + 1):
        for c in range(1, ncols + 1):
            fig.update_xaxes(showgrid=True, row=r, col=c)
            fig.update_yaxes(showgrid=True, row=r, col=c)

    # --- Compute precise annotation positions from axis domains ---
    # Build page_annotations: list of lists (one list per page) of annotation dicts
    page_annotations = []
    # Ensure layout keys are realized: accessing fig.layout[xaxisN] will exist after make_subplots
    for page in range(pages):
        annots = []
        for slot in range(per_page):
            # The axis index for a slot is slot+1 (Plotly numbering starts at 1)
            axis_index = slot + 1
            xaxis_key = f"xaxis{axis_index}"
            yaxis_key = f"yaxis{axis_index}"

            # Default fallback positions if domain info is not available
            center_x = ((slot % ncols) + 0.5) / ncols
            top_y = 1.0 - ((slot // ncols) * (1.0 / nrows))
            y_offset = 0.03

            # Try to read actual domain positions from layout. If missing, keep fallback.
            if xaxis_key in fig.layout and getattr(fig.layout[xaxis_key], "domain", None) is not None:
                dom = fig.layout[xaxis_key].domain
                if isinstance(dom, (list, tuple)) and len(dom) == 2:
                    center_x = (dom[0] + dom[1]) / 2.0

            if yaxis_key in fig.layout and getattr(fig.layout[yaxis_key], "domain", None) is not None:
                ydom = fig.layout[yaxis_key].domain
                if isinstance(ydom, (list, tuple)) and len(ydom) == 2:
                    top_y = ydom[1]

            # sample text for this slot (global index)
            global_idx = page * per_page + slot
            if global_idx < total:
                txt = headers[global_idx]
            else:
                txt = ""

            annots.append(dict(
                x=center_x,
                y=top_y + y_offset,
                xref="paper",
                yref="paper",
                text=txt,
                showarrow=False,
                xanchor="center",
                font=dict(size=12)
            ))
        page_annotations.append(annots)

    # Build per-page visibility arrays for the buttons
    buttons = []
    for page in range(pages):
        visibility = [(meta_page == page) for meta_page, *_ in trace_meta]
        buttons.append(dict(
            label=f"Page {page + 1}",
            method="update",
            args=[{"visible": visibility}, {"annotations": page_annotations[page]}]
        ))

    # Layout: put buttons above the figure (centered)
    fig.update_layout(
        updatemenus=[dict(
            type="buttons",
            buttons=buttons,
            direction="right",
            x=0.5, y=1.14,
            xanchor="center", yanchor="top",
            pad=dict(t=8),
            showactive=True
        )],
        height=900,
        width=1400,
        title=f"CSV Data Viewer: {csv_file}"
    )

    # Initialize with page 0 annotations
    if page_annotations:
        fig.update_layout(annotations=page_annotations[0])

    # Synchronize x-axis zoom across all subplot x-axes (xaxis1..xaxisN)
    for i in range(1, nrows * ncols + 1):
        key = f"xaxis{i}"
        if key in fig.layout:
            try:
                fig.layout[key].matches = "x1"
            except Exception:
                pass

    # pyo.plot(fig, filename="csv_viewer.html", auto_open=True)
    fig.show()


if __name__ == "__main__":
    plot_csv()
