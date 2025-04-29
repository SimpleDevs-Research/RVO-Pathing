# Unity-RVO

This Unity-based repository replicates the operations of the original [van den Berg Reciprocal Velocity Obstacles (RVO) implementation](https://gamma.cs.unc.edu/RVO/) in Unity. Some of this operation is extrapolated from Meng Guo's [RVO_Py_MAS](https://github.com/MengGuo/RVO_Py_MAS) implementation, though it has been cleaned up and optimized for C# and .Net operations. Below is a strict comparison between all three variants:

|Operational Component|**RVO v1.1** (van den Berg)|**RVO_Py_MA**S** (Meng Guo)|**Unity-RVO** (This)|
|:-:|:-|:-|:-|
|Neighbor Search|KDTree (Parallelized)|Global List Iteration|KDTree (Jobs)|
|Penalty Cost|Time To Collision (Parallelized)|Diff. from Desired Velocity|Diff. from Desired Velocity (Jobs)|
|Static Obstacles?|Yes|Yes|No|
|Acceleration during RVO Penalty Cost?|Yes|No|No|
|Acceleration during Agent Update? $^1$|Yes|No|Yes|

$^1$ _Acceleration can be implemented to smoothen movement, making agents move more naturally._

## How to Run Unity-RVO

Follow these instructions:

1. `git clone` this repository into your local system.
2. `git clone` another repository into the `Assets/` folder directly: [https://github.com/SimpleDevs-Tools/Unity-KDTree](https://github.com/SimpleDevs-Tools/Unity-KDTree) (this is not added as a git submodule).
3. Inside of the project, there are several scenes provided in the `Scenes/` folder. They should work out of the box without any modification.
4. To change scene operations, change either the `GenerateAgents` or `GenerateAgentsWithTrajectories` in the inspector. For RVO-specific changes, modify the `Agent` prefab in the `Prefabs/` folder.

---

## Resources

### Relevant References:

1. Jur van den Berg, Ming Lin, and Dinesh Manocha. 2008. Reciprocal Velocity Obstacles for real-time multi-agent navigation. In 2008 IEEE International Conference on Robotics and Automation, 1928–1935. DOI:[https://doi.org/10.1109/ROBOT.2008.4543489](https://doi.org/10.1109/ROBOT.2008.4543489) 
2. Fiorini P, Shiller Z. Motion Planning in Dynamic Environments Using Velocity Obstacles. The International Journal of Robotics Research. 1998;17(7):760-772. doi:[https://doi.org/10.1177/027836499801700706](https://doi.org/10.1177/027836499801700706)
3. Ben Sunshine-Hill. 2019. RVO and ORCA: How they really work. In Game AI Pro 360: Guide to Movement and Pathfinding. CRC Press, 245-256. [https://www.taylorfrancis.com/chapters/edit/10.1201/9780429055096-22/rvo-orca-ben-sunshine-hill](https://www.taylorfrancis.com/chapters/edit/10.1201/9780429055096-22/rvo-orca-ben-sunshine-hill)
4. van den Berg, J., Guy, S.J., Lin, M., Manocha, D. (2011). Reciprocal n-Body Collision Avoidance. In: Pradalier, C., Siegwart, R., Hirzinger, G. (eds) Robotics Research. Springer Tracts in Advanced Robotics, vol 70. Springer, Berlin, Heidelberg. [https://doi.org/10.1007/978-3-642-19457-3_1](https://doi.org/10.1007/978-3-642-19457-3_1)
5. Jamie Snape, Jur van den Berg, Stephen J. Guy, and Dinesh Manocha. 2011. The Hybrid Reciprocal Velocity Obstacle. IEEE Transactions on Robotics 27, 4 (2011), 696-706. DOI:[https://doi.org/10.1109/TRO.2011.2120810](https://doi.org/10.1109/TRO.2011.2120810) 
6. M. Guo and M. M. Zavlanos. 2018. Multirobot Data Gathering Under Buffer Constraints and Intermittent Communication. IEEE Transactions on Robotics 34, 4 (2018), 1082–1097. DOI:[https://doi.org/10.1109/TRO.2018.2830370](https://doi.org/10.1109/TRO.2018.2830370)

### Code Inspiration

1. RVO v1.1 Library: [https://gamma.cs.unc.edu/RVO2/documentation/1.1/](https://gamma.cs.unc.edu/RVO2/documentation/1.1/)
2. Unity KDTree Implementation: [https://github.com/viliwonka/KDTree](https://github.com/viliwonka/KDTree)
3. Unity Markdown Viewer: [https://github.com/gwaredd/UnityMarkdownViewer](https://github.com/gwaredd/UnityMarkdownViewer)

## A Primary on RVO

### Collision Avoidance: Some Context

Reciprocal Velocity Obstacles (RVO) [[1](https://doi.org/10.1109/ROBOT.2008.4543489)] is largely built off  Fiorini and Shiller's "Velocity Obstacles" concept [[2](https://doi.org/10.1177/027836499801700706)]. In basic terms, imagine if you are walking down the street and you see another person coming towards you from the opposite end of the sidewalk. During that moment, you must make a decision on which direction to move towards, in order to avoid colliding with them. A Velocity Obstacle (VO) is a geometric version of that decision, represented usually as a triangular cone starting from yourself towards that other person. Any velocities you choose that end up landing you inside that VO will inevitably lead you to collide with the other person. So in the traditional VO sense, all you have to do is choose a velocity that is outside the VO, and you're guaranteed to avoid hitting that other person.

### Why RVO and not VO's?

In practice, VO's apparently don't work very well. What ends up happening is that maybe in the first frame, two agents heading towards each other will chose to maneuver out of the way of each other in that frame, then they move accordingly (i.e. both agents move left relative to their current heading, thus beginning a synchronous rotation around each other). However, in the next frame, the two agents will then move towards their original heading, which forces them in the next frame to readjust again. In short, you see a kind of "staggering" motion where the collision avoidance trajectory ends up looking like a a bunch of squigglies. Not the smoothest approach. [[3](https://www.taylorfrancis.com/chapters/edit/10.1201/9780429055096-22/rvo-orca-ben-sunshine-hill)]

### What does RVO fix?

van den Berg's RVO implementation attempts to solve this issue by making some assumptions. It modifies the original operation by assuming that all agents are operating under the same collision avoidance strategy, meaning all agents are using the same mentality to avoid hitting one another. They implement some form of prediction as a result - they "slightly" nudge the agent towards an optimal velocity (i.e. they move only halfway towards the optimal velocity) and then perform the VO under the assumption that the other agent has also nudged themselves towards their optimal velocity. This allows for a much smoother avoidance strategy as all agents believe that the other is interested in avoiding collisions.

### Is RVO theoretically guaranteed? Theory vs. Practice

In theory, the proof of RVO guarantees for collision avoidance is valid, but only under the assumption that there are only two agents in the environment with no interruptions - i.e. it's just an open world with no obstacles and as much space as they need. In fact, RVO _should_ technically fail if you introduce a 3rd agent, simply because two agents may end up choosing an optimal velocity that allows them to avoid the 3rd agent... but in doing so they may collide with one another. However, in practice, RVO works much better than expected. There's some logical reasoning behind this:

1. In theory, RVO echoes concepts from gradient-based methods that can be optimized using a cost minimization function (i.e. gradient descent). In a traditional gradient method, the conditions do not change across iteration steps - the primary limiting factor is just the amount of processing time you need to find some minimum or maximum of a curve.
2. However, RVO isn't a **true** gradient-based methodology. In RVO, conditions change with each iteration step: agents will slightly nudge themselves towards (half of) their optimal velocity, and then will have to re-evaluate the optimal velocity. So a generalizable minimization solver isn't possible with RVO, nor should it be expected in practice.
3. Going back to the 3-agent scenario, because the situation has changed from one frame to the next, The two agents that were projected to collide with one another may in fact adjust their optimal velocities to avoid hitting each other while still avoiding the 3rd agent, for example. Over time, the agents will inevitably lead themselves to a happy middle ground where everyone avoids hitting each other, if calibrated properly.

This is why in practice, RVO is sufficiently suitable for basic local collision avoidance modeling in simulations and games.

### Are there other versions of VO's?

You'd want to look at other methods such as van den Berg's "Optimal Reciprocal Collision Avoidance" (ORCA, otherwise known as RVO2) [[4](https://doi.org/10.1007/978-3-642-19457-3_1)] and Hybrid RVO (HRVO) [[5](https://doi.org/10.1109/TRO.2011.2120810)]. However, this repository doesn't explore these fully as RVO is sufficient for our needs.

## Is RVO... Sufficient for Human-Like Behavior?

Good question. There's [this report](https://doi.org/10.1109/IROS.2013.6696726) that talks about RVO variants (ORCA, HRVO, and other variations) in comparison with human behavior, but that doesn't consider raw RVO. We also don't really care about those that propose new updated versions such as [this report](https://doi.org/10.1109/ICRA.2016.7487147), though it would be interesting to look at these in a different project.

Here are some interesting papers I've searched for:

* [this ArXiv report](https://doi.org/10.48550/arXiv.2006.14195) touches not just VOs but also other pathfinding algorithms and such.
* 

