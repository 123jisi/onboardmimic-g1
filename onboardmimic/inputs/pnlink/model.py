"""Required SDK body names in SMPL order; fingers are outside body admission."""
BODY_NAMES = ('Hips', 'LeftUpLeg', 'RightUpLeg', 'Spine', 'LeftLeg', 'RightLeg',
              'Spine1', 'LeftFoot', 'RightFoot', 'Spine2', 'LeftTiptoe', 'RightTiptoe',
              'Neck', 'LeftShoulder', 'RightShoulder', 'Head', 'LeftArm', 'RightArm',
              'LeftForeArm', 'RightForeArm', 'LeftHand', 'RightHand')

# PN firmware 1.4.31.36 omits these terminal joints. The host reference leaves
# their local rotations zero; FK still computes their rest-offset positions.
DERIVED_TERMINALS = frozenset(('LeftTiptoe', 'RightTiptoe'))

# SMPL-X local hand ordering; optional and never part of body admission.
RIGHT_HAND_NAMES = tuple('RightHand'+finger+str(j)
                         for finger in ('Index', 'Middle', 'Pinky', 'Ring', 'Thumb')
                         for j in (1, 2, 3))
LEFT_HAND_NAMES = tuple(name.replace('RightHand', 'LeftHand') for name in RIGHT_HAND_NAMES)
HAND_NAMES = dict(left=LEFT_HAND_NAMES, right=RIGHT_HAND_NAMES)
