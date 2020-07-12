# Max Tree

A utility maximizer library based on a maximum tree structure.

### Motivation

Make it easier to compare and compose algorithms for utility programming.

### Example: Moon Landing

This example is used to improve and push the limits of library.
Why? Because it is fun!

First make it work, then gradually make the model more realistic over time.

- Source: examples/moon.rs
- Description: An experiment to use a greedy optimizer to land a spaceship on the Moon from Earth.
- Status: Simplified model works (without crashing), missing lots of features.
- PRs: Welcome!

![The Moon](https://upload.wikimedia.org/wikipedia/commons/thumb/e/e1/FullMoon2010.jpg/800px-FullMoon2010.jpg)

**If you can make AI land a spaceship on the Moon, then what else is possible?**

### Usage of this library

*Notice! This code is for research purposes only and should NEVER be used in system critical applications!*

**Warning! Improper usage of this library can lead to unsafe AI behavior.**

All algorithms that support general unrestricted classical utility theory are unsafe AI designs.
In particular, for ASI (Artificial Super Intelligence) design,
this library is extremely unsafe without safety verification before use.
For more information about safe ASI core design, see [asi_core0](https://github.com/advancedresearch/asi_core0).

That said, have fun!

### Introduction

A maximum tree stores the maximum utility of node or children for every node.
It is a convenient data structure for comparing or composing different search algorithms.

Once a maximum tree is constructed, searching for the best course of action is trivial.
Since each node stores maximum utility, it is easy to compare children and decide what to do.

- A leaf stores the utility as maximum utility
- A node is terminal if it stores higher maximum utility than its children
- A node's utility is forgotten (overriden) when a child has higher utility

### How to use this library

This library contains only a minimum set of features,
intended to be used as a core for more advanced custom algorithms:

- `Ai::full` does a complete search, finding global maximum
- `Ai::greedy` does a local search, finding local maximum
- `Ai::sub_breadth` constructs children for every available action

The `full` and `greedy` algorithms assumes determinism and perfect information in context.
Basically, it means they should only be used in simulations or controlled environments.

The `Ai::sub_breadth` is used as a common sub-procedure for several algorithms.

For non-determinism, the maximum utility becomes maximum expected utility.
This requires constructing the maximum tree with custom algorithms.
For more information, see "Custom algorithms" below.

### Differences from reward accumulation

A maximum tree does not accumulate rewards over actions.
This means that only the final reward is optimized.

However, it possible to simulate accumulated rewards.

To accumulate rewards, one can use the node data to store utility.
Just add the reward to accumulated rewards so far.
The accumulated rewards are stored as maximum utility.

Optimization for final reward has special terminal semantics.
For more information, see "Terminal semantics" below.

### Discounting action depth

By default, more steps to complete the goal is not penalized.

Subtracting a tiny amount of utility proportional to depth
will make the algorithm prioritize fewer steps to reach the goal.

Since this is common behavior, one can activate this by setting
`AiSettings::eps_depth` to e.g. `0.0000001`.

### Custom algorithms

When the algorithms that are included with this library are too limiting,
it is possible to write custom algorithms that constructs the maximum tree
in other ways or performs different kinds of analysis.

The maximum tree is designed to be convenient for composing different search algorithms.

One can perform e.g. posterior safety analysis without side effects in the context.

It is also possible to restore state of the context and continue search from any node,
using a different search algorithm than the one used to construct the tree.
The final maximum tree can be used with any analysis algorithm.

Under non-determinism or hidden states in the context,
the semantics of maximum utility changes slightly.
This means that one can not expect soundness when composing algorithms
unless the intersecting semantics is sound when exploring a sub-branch.
It is not necessary that the intersecting semantics hold in general,
but it must hold for the particular usage in the application.

The most common use case of this library is for contexts where
there is perfect information and undoing changes restores the environment perfectly.
In most applications, this means simulating the entire world where the AI operates.

### Terminal semantics

Under verification for safety, evaluation of the terminal semantics must be included.
This library is not safe to use when the terminal semantics of a given application
has been not been verified for safety.

When a node is terminal, which is the case for any global maximum
that do not have any children with equal maximum utility,
one must pay careful attention to the semantics of achieving that goal.

In the sense that a global maximum is rearched,
it makes no sense to do so if e.g. the world ends and there nothing left to do.

Reaching infinite utility in an infinitesimal of time does not
correspond to a [Zen Rational](https://github.com/advancedresearch/path_semantics/blob/master/ai-sequences.md#zen-rationality)
human intuition about meaningful goals.
The reason for this is that when the true goal among many possible goals is uncertain,
one risks excluding the true goal with high probability by optimizing for a single goal.
Instead, according to higher order utilitariansim, one should optimize for a cluster of goals
where each goal is reachable from any other.
For more information, see [Groupoid Assumption of Multi-Goal Optimization](https://github.com/advancedresearch/path_semantics/blob/master/papers-wip/groupoid-assumption-of-multi-goal-optimization.pdf).

Although classical utility theory can be used to achieve a single goal,
this does not guarantee that achieving the goal is meaningful.
This is only the case if and only if the true goal is reachable from the achieved goal.
A true goal is defined in [Naive Zen Logic](https://github.com/advancedresearch/path_semantics/blob/master/papers-wip/naive-zen-logic.pdf) as:

```text
true_goal(X) = (goal(X) ? .me) ? me
```

It means, the true goal is the goal I believe I would have if I (the AI agent) were smarter.

If the AI agent achieves global maximum and then self-improve,
in hindsight it was only meaningful to reach global maximum if and only if the new goal is reachable.
Therefore, achieving global maximum is meaningful if and only if the true goal is reachable
from the state of global maximum.

Accumulated rewards can cloud the judgement about terminal semantics.
With accumulated rewards, there is nothing that predicts termination
when the expected utility from the environment is uncertain.
As long as there exists some non-terminating state with positive utility,
there exists a course of action that might increase utility.
Therefore, most AI agents who optimize accumulated rewards do not need to
reason about the terminal semantics in the same way that AI agents that optimizes for final rewards.
However, under self-improvement, accumulated rewards also requires higher order reasoning for safety.

Since optimizing for the final reward is a strict superset of
optimizing for accumulated rewards, the terminal semantics in the first case
is a strict superset of the terminal semantics of the latter.

One can include a term in the final reward that estimates future potential.
If the term for future potential can be negative, then excluding it will lead to unsafety.

## License

Licensed under either of
 * Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)
at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you shall be dual licensed as above, without any
additional terms or conditions.
