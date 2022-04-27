import plotly.express as px
import plotly.graph_objs as go
import plotly.subplots as ps
import pandas as pd
import json
import pathlib
import numpy as np
from typing import List
import tool

colors = px.colors.qualitative.Plotly


def load_trajectory(file_name: str) -> pd.DataFrame:
    """Loads a trajectory from a JSON file and returns it as a pandas DataFrame.

    Args:
        file_name (str): The file name.

    Returns:
        pd.DataFrame: The trajectory as a pandas DataFrame.
    """
    data = {}
    with open(file_name) as json_data:
        data = json.load(json_data)
        data = {
            "sPosition": data["sPosition"],
            "dPosition": data["dPosition"],
            "sVelocity": data["sVelocity"],
            "dVelocity": data["dVelocity"],
            "sAcceleration": data["sAcceleration"],
            "dAcceleration": data["dAcceleration"],
            "heading": data["heading"],
            "curvature": data["curvature"],
            "totalVelocity": data["totalVelocity"],
            "steeringAngle": data["steeringAngle"],
            "totalAcceleration": data["totalAcceleration"],
        }
    return pd.DataFrame.from_dict(data)


def load_data() -> List[pd.DataFrame]:
    """Loads the trajectories from the JSON files. Returns them as a list of pandas DataFrames.

    Returns:
        List[pd.DataFrame]: The trajectories as a list of pandas DataFrames.
    """
    file_paths = sorted(
        pathlib.Path(f"{tool.file_dir}/output/").glob("trajectory_*.json")
    )

    trajectories = []

    for file_path in file_paths:
        trajectories.append(load_trajectory(file_path))

    return trajectories


def plot_trajectories(trajectories: List[pd.DataFrame]) -> None:
    """Plots the actions of the trajectories as well as the resulting trajectories.

    Args:
        trajectories (List[pd.DataFrame]): The trajectories as a list of pandas DataFrames.
    """
    fig = ps.make_subplots(
        rows=1,
        cols=2,
        horizontal_spacing=0.15,
    )

    i = 0
    col = 1
    for trajectory in trajectories:
        d_lateral = trajectory["dPosition"].iloc[-1] - trajectory["dPosition"].iloc[0]
        d_velocity = trajectory["sVelocity"].iloc[-1] - trajectory["sVelocity"].iloc[0]
        trace = go.Scatter(
            name=f"Action {i}",
            x=[d_velocity],
            y=[d_lateral],
            mode="markers",
            marker=dict(color=colors[i]),
            showlegend=False,
        )
        fig.append_trace(trace, 1, col)
        fig["layout"][f"xaxis{col}"]["range"] = [-5.5, 5.5]
        fig["layout"][f"yaxis{col}"]["range"] = [-5.5, 5.5]
        fig["layout"][f"xaxis{col}"]["title"] = tool.l_math(r"\Delta \dot s [m]")
        fig["layout"][f"yaxis{col}"]["title"] = tool.l_math(r"\Delta d [m]")
        i += 1

    i = 0
    col = 2
    for trajectory in trajectories:
        trace = go.Scatter(
            name=f"Trajectory {i}",
            x=trajectory["sPosition"],
            y=trajectory["dPosition"],
            mode="lines",
            line=dict(color=colors[i]),
            showlegend=True,
        )
        fig.append_trace(trace, 1, 2)
        fig["layout"][f"xaxis{col}"]["title"] = tool.l_math(r"s [m]")
        fig["layout"][f"yaxis{col}"]["title"] = tool.l_math(r"d [m]")
        i += 1

    fig.update_layout(
        font=dict(family=tool.font_family, size=tool.font_size),
        template=tool.theme_template,
    )
    tool.generate_output(fig, "trajectory_analysis_action_to_trajectory")


def plot_trajectory_features(
    trajectories: List[pd.DataFrame], features: List[str], y_axis: List[str]
) -> None:
    """Plots the features of the trajectories. The features are the columns of the DataFrame.

    Args:
        trajectories (List[pd.DataFrame]): The trajectories as a list of pandas DataFrames.
        features (List[str]): The list of features to plot.
        y_axis (List[str]): The y axis labels for the features.
    """

    fig = ps.make_subplots(rows=1, cols=len(features), horizontal_spacing=0.15)

    i = 0  # the trajectory index
    for trajectory in trajectories:
        col = 1
        for key in features:
            trace = go.Scatter(
                name=f"Trajectory {i}",
                x=np.linspace(0, 2, len(trajectory[key])),
                y=trajectory[key],
                mode="lines",
                line=dict(color=colors[i]),
                showlegend=True if col == 1 else False,
            )
            fig.append_trace(trace, 1, col)
            fig["layout"][f"xaxis{col}"]["title"] = tool.l_math(r"t [s]")
            fig["layout"][f"yaxis{col}"]["title"] = tool.l_math(y_axis[col - 1])
            col += 1
        i += 1

    fig.update_layout(
        font=dict(family=tool.font_family, size=tool.font_size),
        template=tool.theme_template,
    )
    tool.generate_output(fig, f"trajectory_analysis_{'_'.join(features)}")


if __name__ == "__main__":
    # The tool to run.
    bin = "proseco_planning_tool_trajectory_analysis"
    # The options file to load.
    options = "example_options.json"
    # The scenario file to load.
    scenario = "sc00.json"

    tool.remove_file("trajectory_*.json")
    tool.create_output_dir()
    tool.run_tool(bin, options, scenario)
    trajectories = load_data()
    plot_trajectories(trajectories)

    plot_trajectory_features(
        trajectories, ["sPosition", "dPosition"], [r"s [m/s]", r"d [m/s]"]
    )

    plot_trajectory_features(
        trajectories, ["sVelocity", "dVelocity"], [r"\dot s [m/s]", r"\dot d [m/s]"]
    )

    plot_trajectory_features(
        trajectories,
        ["sAcceleration", "dAcceleration"],
        [r"\ddot s [m/s]", r"\ddot d [m/s]"],
    )

    plot_trajectory_features(
        trajectories, ["curvature", "heading"], [r"c [1/m]", r"\phi [rad]"]
    )

    plot_trajectory_features(
        trajectories, ["totalVelocity", "totalAcceleration"], [r"v [m/s]", r"a [m/s^2]"]
    )
