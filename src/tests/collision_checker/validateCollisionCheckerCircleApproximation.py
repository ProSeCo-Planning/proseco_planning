### Validates the collision checker by drawing the vehicle and checking if it collides with the obstacles in the environment. ###

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib import cm
from matplotlib import patches


##  Draws the vehicle accordingly to its position and orientation
def drawVehicle(
        ax,
        agentID,
        agentsVehicleX,
        agentsVehicleY,
        agentsVehicleHeading,
        agentsVehicleLength,
        agentsVehicleWidth,
):
    ## Specify vehicle
    # p1: Front left
    # p2: Front right
    # p3: Back left
    # p4: Back right

    p1x = (
            agentsVehicleX[0, agentID]
            + agentsVehicleLength[0, agentID] * np.cos(agentsVehicleHeading[0, agentID])
            - agentsVehicleWidth[0, agentID] / 2 * np.sin(agentsVehicleHeading[0, agentID])
    )
    p1y = (
            agentsVehicleY[0, agentID]
            + agentsVehicleLength[0, agentID] * np.sin(agentsVehicleHeading[0, agentID])
            + agentsVehicleWidth[0, agentID] / 2 * np.cos(agentsVehicleHeading[0, agentID])
    )

    p2x = (
            agentsVehicleX[0, agentID]
            + agentsVehicleLength[0, agentID] * np.cos(agentsVehicleHeading[0, agentID])
            + agentsVehicleWidth[0, agentID] / 2 * np.sin(agentsVehicleHeading[0, agentID])
    )
    p2y = (
            agentsVehicleY[0, agentID]
            + agentsVehicleLength[0, agentID] * np.sin(agentsVehicleHeading[0, agentID])
            - agentsVehicleWidth[0, agentID] / 2 * np.cos(agentsVehicleHeading[0, agentID])
    )

    p3x = agentsVehicleX[0, agentID] - agentsVehicleWidth[0, agentID] / 2 * np.sin(
        agentsVehicleHeading[0, agentID]
    )
    p3y = agentsVehicleY[0, agentID] + agentsVehicleWidth[0, agentID] / 2 * np.cos(
        agentsVehicleHeading[0, agentID]
    )

    p4x = agentsVehicleX[0, agentID] + agentsVehicleWidth[0, agentID] / 2 * np.sin(
        agentsVehicleHeading[0, agentID]
    )
    p4y = agentsVehicleY[0, agentID] - agentsVehicleWidth[0, agentID] / 2 * np.cos(
        agentsVehicleHeading[0, agentID]
    )

    xPoints = [p1x, p2x, p4x, p3x, p1x]
    yPoints = [p1y, p2y, p4y, p3y, p1y]
    ax.plot(xPoints, yPoints, "k")
    return ax


## Annotates the graph with the result
def getTitle(collision):
    if collision.size == 1:
        title = "Level 1 - 1 circle, collision: " + str(collision[0]) + "\n"
    elif collision.size == 4:
        title = (
                "Level 1 - 1 circle, collision: "
                + str(collision[0])
                + "\n"
                + "Level 2 - 3 circle, collision: "
                + str(collision[1])
        )
    elif collision.size == 11:
        title = (
                "Level 1 - 1 circle, collision: "
                + str(collision[0])
                + "\n"
                + "Level 2 - 3 circle, collision: "
                + str(collision[1])
                + "\n"
                + "Level 3 - 7 circle, collision: "
                + str(collision[5])
        )
    else:
        title = "something went wrong within annotateResult(ax,collision)"

    return title


## Plot the vehicle and its approximation from file
def plotCurrentVehicleAndCircles(ax, currentEvaluationFile):
    ## Read data from file
    df = pd.read_csv(currentEvaluationFile)
    circleData = df.values
    numberDataElements = circleData.shape[0]

    numberAgentData = 8
    numberAgents = 2

    ## initialize matrizes for collecting the data
    agentsCirclesX = np.empty(shape=[numberDataElements, numberAgents])
    agentsCirclesY = np.empty(shape=[numberDataElements, numberAgents])
    agentsCirclesRadius = np.empty(shape=[numberDataElements, numberAgents])
    agentsVehicleX = np.empty(shape=[numberDataElements, numberAgents])
    agentsVehicleY = np.empty(shape=[numberDataElements, numberAgents])
    agentsVehicleHeading = np.empty(shape=[numberDataElements, numberAgents])
    agentsVehicleLength = np.empty(shape=[numberDataElements, numberAgents])
    agentsVehicleWidth = np.empty(shape=[numberDataElements, numberAgents])

    ## collect the rootNodeData
    for i in range(0, numberAgents):
        # Extract rootNodeData from import
        m_circleX = circleData[:, 0 + i * numberAgentData]
        m_circleY = circleData[:, 1 + i * numberAgentData]
        m_radius = circleData[:, 2 + i * numberAgentData]
        m_vehicleX = circleData[:, 3 + i * numberAgentData]
        m_vehicleY = circleData[:, 4 + i * numberAgentData]
        m_heading = circleData[:, 5 + i * numberAgentData]
        m_vehicleLength = circleData[:, 6 + i * numberAgentData]
        m_vehicleWidth = circleData[:, 7 + i * numberAgentData]

        agentsCirclesX[:, i] = m_circleX
        agentsCirclesY[:, i] = m_circleY
        agentsCirclesRadius[:, i] = m_radius
        agentsVehicleX[:, i] = m_vehicleX
        agentsVehicleY[:, i] = m_vehicleY
        agentsVehicleHeading[:, i] = m_heading
        agentsVehicleLength[:, i] = m_vehicleLength
        agentsVehicleWidth[:, i] = m_vehicleWidth

    ### Extract collision data
    collision = circleData[:, numberAgents * numberAgentData]

    circleColor0 = "b"
    circleColor1 = "g"
    circleLineWidth = 2

    for i in range(0, numberDataElements):
        if collision[i] == 1:
            color0 = "r"
            color1 = "r"
        else:
            color0 = circleColor0
            color1 = circleColor1

        ax.add_patch(
            patches.Circle(
                (agentsCirclesX[i, 0], agentsCirclesY[i, 0]),
                radius=agentsCirclesRadius[i, 0],
                color=color0,
                linewidth=circleLineWidth,
                fill=False,
            )
        )
        ax.add_patch(
            patches.Circle(
                (agentsCirclesX[i, 1], agentsCirclesY[i, 1]),
                radius=agentsCirclesRadius[i, 1],
                color=color1,
                linewidth=circleLineWidth,
                fill=False,
            )
        )

    for i in range(0, 2):
        ax = drawVehicle(
            ax,
            i,
            agentsVehicleX,
            agentsVehicleY,
            agentsVehicleHeading,
            agentsVehicleLength,
            agentsVehicleWidth,
        )

    return ax


## Illusrates the circles drawn for collision checking

## select file to evaluate
directory = "collisionCircleApproximation_debug/"
baseFileName = "debugCircleApproximation"
file_specification = directory + baseFileName
## Change to max index of debugging files
maxIndex = 20

## Font size for plots
m_textsize = 16
m_symbolsize = 8
m_scatterSymbolSize = 50
m_linestyles = ["x", "o", "t", ":"]
m_colors = ("b", "g", "r", "c", "m", "y", "k")
m_colorMap = cm.winter
m_timeColor = cm.cool

## Plot of trajectory of vehicles
fig = plt.figure()
fig.canvas.set_window_title("Collision Check")
ax = fig.add_subplot(111)
for i in range(0, maxIndex + 1):
    currentEvaluationFile = file_specification + str(i) + ".csv"
    ax = plotCurrentVehicleAndCircles(ax, currentEvaluationFile)
ax.set_xlabel("x [m]", fontsize=m_textsize)
ax.set_ylabel("y [m]", fontsize=m_textsize)
# ax.set_title(getTitle(collision), fontsize = m_textsize)
ax.grid(True)
ax.axis("equal")

## Plot of final collision
fig = plt.figure()
fig.canvas.set_window_title("Collision Check")
ax = fig.add_subplot(111)
currentEvaluationFile = file_specification + str(maxIndex) + ".csv"
ax = plotCurrentVehicleAndCircles(ax, currentEvaluationFile)
ax.set_xlabel("x [m]", fontsize=m_textsize)
ax.set_ylabel("y [m]", fontsize=m_textsize)
ax.set_title("detailed view of collision", fontsize=m_textsize)
ax.grid(True)
ax.axis("equal")

plt.show()
