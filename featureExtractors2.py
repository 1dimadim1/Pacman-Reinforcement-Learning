# featureExtractors.py
# --------------------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"Feature extractors for Pacman game states"

from game import Directions, Actions
import util

class FeatureExtractor:
    def getFeatures(self, state, action):
        """
          Returns a dict from features to counts
          Usually, the count will just be 1.0 for
          indicator functions.
        """
        util.raiseNotDefined()

class IdentityExtractor(FeatureExtractor):
    def getFeatures(self, state, action):
        feats = util.Counter()
        feats[(state,action)] = 1.0
        return feats

class CoordinateExtractor(FeatureExtractor):
    def getFeatures(self, state, action):
        feats = util.Counter()
        feats[state] = 1.0
        feats['x=%d' % state[0]] = 1.0
        feats['y=%d' % state[0]] = 1.0
        feats['action=%s' % action] = 1.0
        return feats

def closestFood(pos, food, walls):
    """
    closestFood -- this is similar to the function that we have
    worked on in the search project; here its all in one place
    """
    fringe = [(pos[0], pos[1], 0)]
    expanded = set()
    while fringe:
        pos_x, pos_y, dist = fringe.pop(0)
        if (pos_x, pos_y) in expanded:
            continue
        expanded.add((pos_x, pos_y))
        # if we find a food at this location then exit
        if food[pos_x][pos_y]:
            return dist
        # otherwise spread out from the location to its neighbours
        nbrs = Actions.getLegalNeighbors((pos_x, pos_y), walls)
        for nbr_x, nbr_y in nbrs:
            fringe.append((nbr_x, nbr_y, dist+1))
    # no food found
    return None

def closestCapsule(pos, capsules, walls):
    fringe = [(pos[0], pos[1], 0)]
    checked = set()
    while fringe:
        pos_x, pos_y, dist = fringe.pop(0)
        if (pos_x, pos_y) in checked:
            continue
        checked.add((pos_x, pos_y))

        for capsule_x, capsule_y in capsules:
            if pos_x == capsule_x and pos_y == capsule_y:
                return dist

        nbrs = Actions.getLegalNeighbors((pos_x, pos_y), walls)
        for nbr_x, nbr_y in nbrs:
            fringe.append((nbr_x, nbr_y, dist+1))
    # no capsule found
    return None

def closestGhost(pos, ghosts, walls):
    fringe = [(pos[0], pos[1], 0)]
    checked = set()
    while fringe:
        pos_x, pos_y, dist = fringe.pop(0)
        if (pos_x, pos_y) in checked:
            continue
        checked.add((pos_x, pos_y))

        for ghost in ghosts:
            ghost_x, ghost_y = ghost.getPosition()
            if pos_x == ghost_x and pos_y == ghost_y:
                return dist, ghost.scaredTimer

        nbrs = Actions.getLegalNeighbors((pos_x, pos_y), walls)
        for nbr_x, nbr_y in nbrs:
            fringe.append((nbr_x, nbr_y, dist+1))
    # no capsule found
    return None, None

class SimpleExtractor(FeatureExtractor):
    """
    Returns simple features for a basic reflex Pacman:
    - whether food will be eaten
    - how far away the next food is
    - whether a ghost collision is imminent
    - whether a ghost is one step away
    """

    def getFeatures(self, state, action):
        # extract the grid of food and wall locations and get the ghost locations
        food = state.getFood()
        walls = state.getWalls()
        ghosts = state.getGhostPositions()

        features = util.Counter()

        features["bias"] = 1.0

        # compute the location of pacman after he takes the action
        x, y = state.getPacmanPosition()
        dx, dy = Actions.directionToVector(action)
        next_x, next_y = int(x + dx), int(y + dy)

        # count the number of ghosts 1-step away
        features["#-of-ghosts-1-step-away"] = sum((next_x, next_y) in Actions.getLegalNeighbors(g, walls) for g in ghosts)

        # if there is no danger of ghosts then add the food feature
        if not features["#-of-ghosts-1-step-away"] and food[next_x][next_y]:
            features["eats-food"] = 1.0

        dist = closestFood((next_x, next_y), food, walls)
        if dist is not None:
            # make the distance a number less than one otherwise the update
            # will diverge wildly
            features["closest-food"] = float(dist) / (walls.width * walls.height)
        features.divideAll(10.0)
        # features["#-of-ghosts-1-step-away"] = features["#-of-ghosts-1-step-away"] / 10.0
        return features

class MyExtractor(FeatureExtractor):
    """
    Returns simple features for a basic reflex Pacman:
    - whether food will be eaten
    - how far away the next food is
    - whether a ghost collision is imminent
    - whether a ghost is one step away
    - mb add food remaining
    - ghost eat giving pts
    """

    def getFeatures(self, state, action):
        food = state.getFood()
        walls = state.getWalls()
        ghosts = state.getGhostPositions()

        # print dir(state)
        # print "food:", food
        # print "num food", state.getNumFood()
        capsules = state.getCapsules()

        features = util.Counter()

# ['__doc__', '__eq__', '__hash__', '__init__', '__module__', '__str__', 'data', 'deepCopy', 'explored', 'generatePacmanSuccessor', 'generateSuccessor', 'getAndResetExplored', 'getCapsules', 'getFood', 'getGhostPosition', 'getGhostPositions', 'getGhostState', 'getGhostStates', 'getLegalActions', 'getLegalPacmanActions', 'getNumAgents', 'getNumFood', 'getPacmanPosition', 'getPacmanState', 'getScore', 'getWalls', 'hasFood', 'hasWall', 'initialize', 'isLose', 'isWin']
        
        x, y = state.getPacmanPosition()
        dx, dy = Actions.directionToVector(action)
        next_x, next_y = int(x + dx), int(y + dy)

        # features["capsules_count"] = len(capsules)

        # if len(capsules) > 0:
        #     capsule_dist = closestCapsule((next_x, next_y), capsules, walls)
        #     features["nearest_capsule"] = float(capsule_dist) / (walls.width * walls.height)


        ghost_states = state.getGhostStates()
        active_ghosts = []
        scared_ghosts = []
        for ghost in ghost_states:
            if ghost.scaredTimer > 0:
                scared_ghosts.append(ghost)
            else:
                active_ghosts.append(ghost)

        # if len(active_ghosts) > 0:
        #     dist, scared_timer = closestGhost((next_x, next_y), active_ghosts, walls)
        #     if dist is not None:
        #         features['closest_active_ghost'] = float(dist) / (walls.width * walls.height)

        if len(scared_ghosts) > 0:
            dist, scared_timer = closestGhost((next_x, next_y), scared_ghosts, walls)
            if dist is not None:
                features['closest_scared_ghost'] = float(dist) / (walls.width * walls.height)
                features['closest_scared_ghost_timer'] = scared_timer / 4

        # features['num_food'] = float(state.getNumFood()) / (walls.width * walls.height)

        
        features["active_ghosts_near"] = sum((next_x, next_y) in Actions.getLegalNeighbors(g.getPosition(), walls) for g in active_ghosts)
        features["scared_ghosts_near"] = sum((next_x, next_y) in Actions.getLegalNeighbors(g.getPosition(), walls) for g in scared_ghosts)

        # features["active_ghosts_count"] = len(active_ghosts)
        # features["scared_ghosts_count"] = len(scared_ghosts)

        # if len(ghosts) == len(scared_ghosts):
        #     features["all_scared"] = 1.0

        # if len(ghosts) == len(active_ghosts):
        #     features["all_active"] = 1.0

        # if there is no danger of ghosts then add the food feature
        if not features["active_ghosts_near"] and food[next_x][next_y]:
            features["eats-food"] = 1.0

        dist = closestFood((next_x, next_y), food, walls)
        if dist is not None:
            # make the distance a number less than one otherwise the update
            # will diverge wildly
            features["closest-food"] = float(dist) / (walls.width * walls.height)
        features.divideAll(10.0)
        # features["#-of-ghosts-1-step-away"] = features["#-of-ghosts-1-step-away"] / 10.0
        return features


class MyExtractor2(FeatureExtractor):
    """
    Returns simple features for a basic reflex Pacman:
    - whether food will be eaten
    - how far away the next food is
    - whether a ghost collision is imminent
    - whether a ghost is one step away
    """

    def getFeatures(self, state, action):
        # extract the grid of food and wall locations and get the ghost locations
        food = state.getFood()
        walls = state.getWalls()
        ghosts = state.getGhostPositions()

        features = util.Counter()

        # features["bias"] = 1.0

        # compute the location of pacman after he takes the action
        x, y = state.getPacmanPosition()
        dx, dy = Actions.directionToVector(action)
        next_x, next_y = int(x + dx), int(y + dy)

        # count the number of ghosts 1-step away
        ghosts_near = sum((next_x, next_y) in Actions.getLegalNeighbors(g, walls) for g in ghosts)

        # features["#-of-ghosts-1-step-away"] = sum((next_x, next_y) in Actions.getLegalNeighbors(g, walls) for g in ghosts)
        
        is_near = 0
        if ghosts_near > 0:
            is_near = 1
        features["is_ghost_near"] = is_near

        # if there is no danger of ghosts then add the food feature
        if not features["is_ghost_near"] and food[next_x][next_y]:
            features["eats-food"] = 1.0

        dist = closestFood((next_x, next_y), food, walls)
        if dist is not None:
            # make the distance a number less than one otherwise the update
            # will diverge wildly
            features["closest-food"] = float(dist) / (walls.width * walls.height)
        features.divideAll(10.0)
        # features["#-of-ghosts-1-step-away"] = features["#-of-ghosts-1-step-away"] / 10.0
        return features