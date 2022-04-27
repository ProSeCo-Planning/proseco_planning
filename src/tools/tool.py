"""Collection of functions used for the tools."""

import rospkg
import plotly.io as pio
import plotly.graph_objects as go
from pathlib import Path
import subprocess

###############################################################################
# Configuration variables, which can be changed by the user.
#
# The file format to be used for the plots, e.g. png, pdf, eps, svg.
file_format = "png"
# The font size to be used for the plots.
font_size = 18
# The font family to be used for the plots.
font_family = "Computer Modern"
# The theme to be used for the plots.
theme_template = "plotly_white"
# Whether or not to create html output.
export_html = False
###############################################################################

# The directory of the current file.
file_dir = Path(__file__).parent.resolve()

def l_math(string: str) -> str:
    """Returns a LaTeX math mode string.

    Args:
        string (str): The string to be converted to LaTeX math mode.

    Returns:
        str: The LaTeX math mode string.
    """
    if(file_format == "svg"):
        return f"&#36;{string}&#36;"
    elif(file_format == "png" or export_html):
        return f"${string}$"

def remove_file(file_name: str) -> None:
    """Removes all files from the output directory that match the specified file name.

    Args:
        file_name (str): The name of the file to be removed.
    """
    file_paths = Path(f"{file_dir}/output/").glob(file_name)

    for file_path in file_paths:
        file_path.unlink()

def generate_output(fig: go.Figure, file_name: str) -> None:
    # fig.show()
    fig.write_image(f"{file_dir}/output/{file_name}.{file_format}")
    if(export_html):
        pio.write_html(fig, f"{file_dir}/output/{file_name}.html", include_plotlyjs="cdn", include_mathjax="cdn") 

def create_output_dir() -> None:
    """Creates the output directory for all tools."""
    output = Path(f"{file_dir}/output")
    try:
        output.mkdir(exist_ok=True)
    except:
        pass


def run_tool(tool: str, options: str, scenario: str) -> None:
    """Runs a C++ tool with the specified options and scenario files.

    Args:
        tool (str): The C++ tool to run.
        options (str): The options to load into the tool.
        scenario (str): The scenario to load into the tool.
    """
    # The path of the ros_proseco_planning package.
    pack_path = rospkg.RosPack().get_path("ros_proseco_planning")
    # The path of the proseco workspace.
    ws_path = Path(pack_path).parents[1]
    cmd = f"cd {ws_path} && ./devel_isolated/proseco_planning/lib/proseco_planning/{tool} {pack_path}/config/options/{options} {pack_path}/config/scenarios/{scenario} {str(file_dir)}/output"
    # Execute the bash command.
    subprocess.run(cmd, shell=True, check=True)