#![deny(missing_docs)]

//! # Max Tree
//!
//! A utility maximizer library based on a maximum tree structure.
//!
//! ### Motivation
//!
//! Make it easier to compare and compose algorithms for utility programming.
//!
//! ### Example: Moon Landing
//!
//! This example is used to improve and push the limits of library.
//! Why? Because it is fun!
//!
//! First make it work, then gradually make the model more realistic over time.
//!
//! - Source: examples/moon.rs
//! - Description: An experiment to use a greedy optimizer to land a spaceship on the Moon from Earth.
//! - Status: Simplified model works (without crashing), missing lots of features.
//! - PRs: Welcome!
//!
//! ![The Moon](https://upload.wikimedia.org/wikipedia/commons/thumb/e/e1/FullMoon2010.jpg/800px-FullMoon2010.jpg)
//!
//! **If you can make AI land a spaceship on the Moon, then what else is possible?**
//!
//! ### Usage of this library
//!
//! *Notice! This code is for research purposes only and should NEVER be used in system critical applications!*
//!
//! **Warning! Improper usage of this library can lead to unsafe AI behavior.**
//!
//! All algorithms that support general unrestricted classical utility theory are unsafe AI designs.
//! In particular, for ASI (Artificial Super Intelligence) design,
//! this library is extremely unsafe without safety verification before use.
//! For more information about safe ASI core design, see [asi_core0](https://github.com/advancedresearch/asi_core0).
//!
//! That said, have fun!
//!
//! ### Introduction
//!
//! A maximum tree stores the maximum utility of node or children for every node.
//! It is a convenient data structure for comparing or composing different search algorithms.
//!
//! Once a maximum tree is constructed, searching for the best course of action is trivial.
//! Since each node stores maximum utility, it is easy to compare children and decide what to do.
//!
//! - A leaf stores the utility as maximum utility
//! - A node is terminal if it stores higher maximum utility than its children
//! - A node's utility is forgotten (overriden) when a child has higher utility
//!
//! ### How to use this library
//!
//! This library contains only a minimum set of features,
//! intended to be used as a core for more advanced custom algorithms:
//!
//! - `Ai::full` does a complete search, finding global maximum
//! - `Ai::greedy` does a local search, finding local maximum
//! - `Ai::sub_breadth` constructs children for every available action
//!
//! The `full` and `greedy` algorithms assumes determinism and perfect information in context.
//! Basically, it means they should only be used in simulations or controlled environments.
//!
//! The `Ai::sub_breadth` is used as a common sub-procedure for several algorithms.
//!
//! For non-determinism, the maximum utility becomes maximum expected utility.
//! This requires constructing the maximum tree with custom algorithms.
//! For more information, see "Custom algorithms" below.
//!
//! ### Differences from reward accumulation
//!
//! A maximum tree does not accumulate rewards over actions.
//! This means that only the final reward is optimized.
//!
//! However, it possible to simulate accumulated rewards.
//!
//! To accumulate rewards, one can use the node data to store utility.
//! Just add the reward to accumulated rewards so far.
//! The accumulated rewards are stored as maximum utility.
//!
//! Optimization for final reward has special terminal semantics.
//! For more information, see "Terminal semantics" below.
//!
//! ### Discounting action depth
//!
//! By default, more steps to complete the goal is not penalized.
//!
//! Subtracting a tiny amount of utility proportional to depth
//! will make the algorithm prioritize fewer steps to reach the goal.
//!
//! Since this is common behavior, one can activate this by setting
//! `AiSettings::eps_depth` to e.g. `0.0000001`.
//!
//! ### Custom algorithms
//!
//! When the algorithms that are included with this library are too limiting,
//! it is possible to write custom algorithms that constructs the maximum tree
//! in other ways or performs different kinds of analysis.
//!
//! The maximum tree is designed to be convenient for composing different search algorithms.
//!
//! One can perform e.g. posterior safety analysis without side effects in the context.
//!
//! It is also possible to restore state of the context and continue search from any node,
//! using a different search algorithm than the one used to construct the tree.
//! The final maximum tree can be used with any analysis algorithm.
//!
//! Under non-determinism or hidden states in the context,
//! the semantics of maximum utility changes slightly.
//! This means that one can not expect soundness when composing algorithms
//! unless the intersecting semantics is sound when exploring a sub-branch.
//! It is not necessary that the intersecting semantics hold in general,
//! but it must hold for the particular usage in the application.
//!
//! The most common use case of this library is for contexts where
//! there is perfect information and undoing changes restores the environment perfectly.
//! In most applications, this means simulating the entire world where the AI operates.
//!
//! ### Terminal semantics
//!
//! Under verification for safety, evaluation of the terminal semantics must be included.
//! This library is not safe to use when the terminal semantics of a given application
//! has been not been verified for safety.
//!
//! When a node is terminal, which is the case for any global maximum
//! that do not have any children with equal maximum utility,
//! one must pay careful attention to the semantics of achieving that goal.
//!
//! In the sense that a global maximum is rearched,
//! it makes no sense to do so if e.g. the world ends and there nothing left to do.
//!
//! Reaching infinite utility in an infinitesimal of time does not
//! correspond to a [Zen Rational](https://github.com/advancedresearch/path_semantics/blob/master/ai-sequences.md#zen-rationality)
//! human intuition about meaningful goals.
//! The reason for this is that when the true goal among many possible goals is uncertain,
//! one risks excluding the true goal with high probability by optimizing for a single goal.
//! Instead, according to higher order utilitariansim, one should optimize for a cluster of goals
//! where each goal is reachable from any other.
//! For more information, see [Groupoid Assumption of Multi-Goal Optimization](https://github.com/advancedresearch/path_semantics/blob/master/papers-wip/groupoid-assumption-of-multi-goal-optimization.pdf).
//!
//! Although classical utility theory can be used to achieve a single goal,
//! this does not guarantee that achieving the goal is meaningful.
//! This is only the case if and only if the true goal is reachable from the achieved goal.
//! A true goal is defined in [Naive Zen Logic](https://github.com/advancedresearch/path_semantics/blob/master/papers-wip/naive-zen-logic.pdf) as:
//!
//! ```text
//! true_goal(X) = (goal(X) ? .me) ? me
//! ```
//!
//! It means, the true goal is the goal I believe I would have if I (the AI agent) were smarter.
//!
//! If the AI agent achieves global maximum and then self-improve,
//! in hindsight it was only meaningful to reach global maximum if and only if the new goal is reachable.
//! Therefore, achieving global maximum is meaningful if and only if the true goal is reachable
//! from the state of global maximum.
//!
//! Accumulated rewards can cloud the judgement about terminal semantics.
//! With accumulated rewards, there is nothing that predicts termination
//! when the expected utility from the environment is uncertain.
//! As long as there exists some non-terminating state with positive utility,
//! there exists a course of action that might increase utility.
//! Therefore, most AI agents who optimize accumulated rewards do not need to
//! reason about the terminal semantics in the same way that AI agents that optimizes for final rewards.
//! However, under self-improvement, accumulated rewards also requires higher order reasoning for safety.
//!
//! Since optimizing for the final reward is a strict superset of
//! optimizing for accumulated rewards, the terminal semantics in the first case
//! is a strict superset of the terminal semantics of the latter.
//!
//! One can include a term in the final reward that estimates future potential.
//! If the term for future potential can be negative, then excluding it will lead to unsafety.
//!
//! ## License
//!
//! Licensed under either of
//!  * Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
//!  * MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)
//! at your option.
//!
//! ### Contribution
//!
//! Unless you explicitly state otherwise, any contribution intentionally submitted
//! for inclusion in the work by you shall be dual licensed as above, without any
//! additional terms or conditions.

/// Reexports commonly used objects.
pub mod prelude {
    pub use super::{Ai, AiAnalysis, AiSettings, Node};
}

/// Stores action node (represented as a maximum tree).
///
/// Each node stores a maximum utility of itself or any children.
///
/// A terminal node has higher utility than any other children.
#[derive(Debug)]
pub struct Node<T, A> {
    /// Stores maximum utility of itself or any children.
    pub max: f64,
    /// Stores node data.
    pub data: T,
    /// Stores child nodes.
    ///
    /// Each child has an associated action.
    /// The associated action identifies the node.
    /// This means the action should be unique among the children.
    /// This invariant is enforced by trusted search algorithms.
    /// Use `check_unique_actions` when the input is not trusted.
    pub children: Vec<(A, Node<T, A>)>,
}

impl<T, A> Node<T, A> {
    /// Creates a new root.
    ///
    /// This sets the utility to `NaN` (not a number).
    /// There are no children, which must be added through search.
    pub fn root(data: T) -> Node<T, A> {
        Node {
            max: std::f64::NAN,
            data,
            children: vec![]
        }
    }

    /// Returns `true` if all actions among children are unique.
    ///
    /// This algorithm does not provide any proof of the collision among children,
    /// since this use case is uncommon (the invariant is enforced by search algorithms).
    pub fn check_unique_actions(&self) -> bool
        where A: Eq + std::hash::Hash
    {
        use std::collections::HashSet;

        let mut hash_set = HashSet::new();
        for &(ref a, _) in &self.children {
            if hash_set.contains(a) {return false}
            hash_set.insert(a.clone());
        }
        true
    }

    /// Returns `true` if the node is terminal.
    ///
    /// A terminal node has no children with equal or greater utility.
    /// This means that without paying attention to terminal semantics,
    /// an unsafe AI design can lead to a "stranded" situation.
    /// For more information, see the section "Terminal semantics"
    /// at this crate's top level documentation.
    pub fn terminal(&self) -> bool {self.optimal().is_none()}

    /// Returns the optimal course of action, if any.
    ///
    /// Returns `None` if no children has higher utility.
    /// This means the node is terminal.
    pub fn optimal(&self) -> Option<usize> {
        for (i, ch) in self.children.iter().enumerate() {
            if ch.1.max >= self.max {return Some(i)}
        }
        None
    }

    /// Returns optimal path from root.
    pub fn optimal_path(&self) -> Vec<usize> {
        let mut node = self;
        let mut res = vec![];
        loop {
            if let Some(i) = node.optimal() {
                node = &node.children[i].1;
                res.push(i);
            } else {
                break;
            }
        }
        res
    }
}

/// AI settings.
pub struct AiSettings {
    /// Maximum depth.
    pub max_depth: usize,
    /// Utility discount from action depth.
    ///
    /// This is usually a small positive number (e.g. `0.000001`).
    pub eps_depth: f64,
    /// Whether to run analysis.
    pub analysis: bool,
    /// Eliminate unexplored actions when using greedy search.
    pub greed_elim: bool,
    /// A limit to estimated memory usage,
    /// causing the search to terminate.
    ///
    /// This limit is only checked occationally, e.g. after breadth search,
    /// so actual memory usage before termination will exceed limit.
    pub max_mib: Option<f64>,
}

impl AiSettings {
    /// Creates new settings.
    pub fn new(max_depth: usize, eps_depth: f64) -> AiSettings {
        AiSettings {
            max_depth,
            eps_depth,
            analysis: false,
            greed_elim: true,
            max_mib: None,
        }
    }
}

/// Stores results from analysis.
pub struct AiAnalysis {
    /// Keeps track of maximum number of nodes.
    pub node_count: usize,
}

impl AiAnalysis {
    /// Creates new AI analysis.
    pub fn new() -> AiAnalysis {
        AiAnalysis {
            node_count: 0,
        }
    }
}

impl AiAnalysis {
    /// Estimates the maximum memory usage of nodes in Gibibytes.
    pub fn gib(&self, node_size: usize) -> f64 {
        (self.node_count as f64 * node_size as f64) / 1073741824.0
    }

    /// Estimates the maximum memory usage of nodes in Mibibytes.
    pub fn mib(&self, node_size: usize) -> f64 {
        (self.node_count as f64 * node_size as f64) / 1048576.0
    }

    /// Estimates the maximum memory usage of nodes in Kibibytes.
    pub fn kib(&self, node_size: usize) -> f64 {
        (self.node_count as f64 * node_size as f64) / 1024.0
    }
}

/// AI setup.
///
/// Provides a common setup for different search algorithms.
/// The search algorithm constructs a maximum tree,
/// which can be used to find the optimal course of action.
///
/// The `T` parameter is the type of node data.
/// This is used to store delta changes for undo operations.
/// Also stores data that tracks internal state of the AI agent,
/// when the AI agent is not interacting with the context (pure search).
/// Node data is stored separately from the context, making it easy
/// to analyse after constructing the maximum tree.
///
/// The `A` parameter is the type of action.
/// It describes the choices the AI agent can make in a specific context.
/// An action modifies the context and must be undone when rolling back changes.
///
/// The `C` parameter is the type of context (environment).
/// This stores the data that is necessary to calculate utility.
/// The context is modified by actions when exploring,
/// but these changes are undone when rolling back changes.
pub struct Ai<T, A, C> {
    /// Calculates utility from data and context.
    pub utility: fn(&T, &C) -> f64,
    /// Returns a list of possible actions.
    pub actions: fn(&T, &C) -> Vec<A>,
    /// Executes an action, returning new node data.
    pub execute: fn(&T, a: &A, &mut C) -> Result<T, ()>,
    /// Undoes change made to context.
    ///
    /// The data required to rollback delta changes
    /// must be stored in node data.
    pub undo: fn(&T, &mut C),
    /// Stores AI settings.
    pub settings: AiSettings,
    /// Stores analysis.
    pub analysis: AiAnalysis,
}

impl<T, A, C> Ai<T, A, C> {
    /// Computes the size of nodes in bytes.
    pub fn node_size(&self) -> usize {
        std::mem::size_of::<Node<T, A>>()
    }

    /// Calculates utility with extra terms computed from settings.
    pub fn utility_with_settings(&self, data: &T, depth: usize, ctx: &C) -> f64 {
        let utility = (self.utility)(data, ctx);
        let discount_step = -self.settings.eps_depth * depth as f64;
        utility + discount_step
    }

    /// Updates context by tracing the optimal path.
    pub fn update<'a>(&mut self, node: &'a Node<T, A>, ctx: &mut C) -> Option<usize> {
        if let Some(i) = node.optimal() {
            if (self.execute)(&node.data, &node.children[i].0, ctx).is_ok() {
                Some(i)
            } else {
                None
            }
        } else {
            None
        }
    }

    /// A sub-procedure constructing maximum tree of all available actions.
    ///
    /// Uses by other search algorithms.
    pub fn sub_breadth(&mut self, root: &mut Node<T, A>, depth: usize, ctx: &mut C)
        where A: Clone
    {
        root.children.clear();
        let actions = (self.actions)(&root.data, ctx);
        for a in &actions {
            if let Ok(data) = (self.execute)(&root.data, a, ctx) {
                let utility = self.utility_with_settings(&data, depth + 1, ctx);
                if utility > root.max {
                    root.max = utility;
                }

                // Undo changes made to context to reset state.
                (self.undo)(&data, ctx);

                root.children.push((a.clone(), Node {
                    max: utility,
                    data,
                    children: vec![],
                }));

                if self.settings.analysis {
                    self.analysis.node_count += 1;
                }
            }
        }
    }

    /// Returns `true` when estimated memory usage is exceeded, `false` otherwise.
    ///
    /// Returns `false` when analysis is deactivated.
    pub fn memory_exceeded(&self) -> bool {
        if self.settings.analysis {
            if let Some(limit) = self.settings.max_mib {
                self.analysis.mib(self.node_size()) >= limit
            } else {false}
        } else {false}
    }

    /// Only picks choices that increases utility.
    ///
    /// In order to find global maximum, it requires utility gradient to be convex.
    pub fn greedy(&mut self, root: &mut Node<T, A>, depth: usize, ctx: &mut C)
        where A: Clone
    {
        if root.max.is_nan() {
            root.max = self.utility_with_settings(&root.data, depth, ctx);
        }

        self.sub_breadth(root, depth, ctx);

        if depth >= self.settings.max_depth {return};
        if self.memory_exceeded() {return};

        if let Some(i) = root.optimal() {
            let i = if self.settings.greed_elim {
                if self.settings.analysis {
                    self.analysis.node_count -= root.children.len() - 1;
                }
                root.children.swap(i, 0);
                root.children.truncate(1);
                0
            } else {i};

            let a = &root.children[i].0;
            if let Ok(_) = (self.execute)(&root.data, a, ctx) {
                let ch = &mut root.children[i].1;
                self.greedy(ch, depth + 1, ctx);

                // Undo changes made to context to reset state.
                (self.undo)(&ch.data, ctx);

                // Update maximum utility since children are changed.
                if ch.max > root.max {
                    root.max = ch.max;
                }
            }
        }
    }

    /// Performs a full construction of the entire maximum tree.
    pub fn full(&mut self, root: &mut Node<T, A>, depth: usize, ctx: &mut C)
        where A: Clone
    {
        if root.max.is_nan() {
            root.max = self.utility_with_settings(&root.data, depth, ctx);
        }

        self.sub_breadth(root, depth, ctx);

        if depth >= self.settings.max_depth {return};
        if self.memory_exceeded() {return};

        for (ref a, ref mut ch) in &mut root.children {
            if let Ok(_) = (self.execute)(&root.data, a, ctx) {
                self.full(ch, depth + 1, ctx);

                // Undo changes made to context to reset state.
                (self.undo)(&ch.data, ctx);

                // Update maximum utility since children are changed.
                if ch.max > root.max {
                    root.max = ch.max;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
