package frc.robot.commands.paths;

import easypath.FollowPath;
import easypath.Path;

public class Test extends FollowPath {
    public Test() {
        super(new Path(t -> 
                /* {"start":{"x":50,"y":164},"mid1":{"x":91,"y":160},"mid2":{"x":154,"y":176},"end":{"x":155,"y":140}} */
                (-216 * Math.pow(t, 2) + 120 * t + -12) / (-252 * Math.pow(t, 2) + 132 * t + 123),
                117.375)
                , x -> {
                    if (x < 0.15) return 0.6;
                    else if (x < 0.75) return 0.8;
                    else return 0.25;
        });
    }
}
