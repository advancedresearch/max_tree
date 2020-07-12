/*
A simple example testing full search.
*/

use max_tree::prelude::*;

type Map = Vec<Vec<u8>>;
type Pos = [usize; 2];

fn main() {
    let ref mut map: Map = vec![
        vec![0, 0, 1],
        vec![0, 0, 0],
        vec![0, 0, 0]
    ];
    let start: Pos = [0, 2];

    let max_depth = 4;
    let eps_depth = 0.00001;
    let mut ai = Ai {
        actions: actions,
        execute: execute,
        utility: utility,
        undo: undo,
        settings: AiSettings::new(max_depth, eps_depth),
        analysis: AiAnalysis::new(),
    };
    let mut root = Node::root(start);
    ai.full(&mut root, 0, map);

    let mut node = &root;
    loop {
        println!("{:?}", node.data);
        if let Some(i) = node.optimal() {
            node = &node.children[i].1;
        } else {
            break;
        }
    }
}

fn undo(_: &Pos, _: &mut Map) {}

fn utility(pos: &Pos, map: &Map) -> f64 {
    map[pos[1]][pos[0]] as f64
}

fn execute(pos: &Pos, action: &Action, map: &mut Map) -> Result<[usize; 2], ()> {
    Ok(match *action {
        Action::Left => {
            if pos[0] <= 0 {return Err(())};
            [pos[0] - 1, pos[1]]
        }
        Action::Right => {
            if pos[0] + 1 >= map[0].len() {return Err(())};
            [pos[0] + 1, pos[1]]
        }
        Action::Up => {
            if pos[1] <= 0 {return Err(())};
            [pos[0], pos[1] - 1]
        }
        Action::Down => {
            if pos[1] + 1 >= map.len() {return Err(())};
            [pos[0], pos[1] + 1]
        }
    })
}

fn actions(_: &Pos, _: &Map) -> Vec<Action> {
    vec![Action::Left, Action::Right, Action::Up, Action::Down]
}

#[derive(Clone, Copy, Debug)]
pub enum Action {
    Left,
    Right,
    Up,
    Down,
}
