# ch-router

A router based on Contraction Hierarchies with optional path unfolding and "hot groups".

## CH creation

To create the Contraction Hierarchies you need to provide a list of nodes and edges:

```rust
let nodes = [Node { x: 0.0, y: 0.0 }, ...];

let edges = [
    Edge {
        from: 0,
        to: 1,
        weight: 2.0,
    },
    ...
];

let max_speed = 30.0; // pass f32::MAX if unknown

let ch = ContractionHierarchy::new(&nodes, &edges, max_speed);
```

## Routing

Basic routing is done with the `route` mehtod:

```rust
let Route { path, distance } = router.route(0, 1).unwrap();
```

If you don't need the actual path, you can use the `distance` method (it is recommended to disable the `path-unfolding` feature in this case):

```rust
let distance = router.distance(0, 1).unwrap();
```

Both methods are also available on the `Router` struct, which will reuse allocations, so it may be faster for multiple queries:

```rust
let mut router = Router::new(&ch);

let distance = router.distance(0, 1).unwrap();
let route = router.route(0, 1).unwrap();
```

## Hot groups

Hot groups are a way to greatly accelerate routing queries for a subset of nodes. This comes at a much larger memory cost per node, because it caches the bidirectional dijkstra results, but will yield about 100x faster queries. Hot groups can be created with the `create_hot_group` method by passing the desired subset of nodes:

```rust
let hot_group = ch.create_hot_group(&[0, 1, 2]);
```

Distance can then be queried within the group:

```rust
let distance = hot_group.distance(0, 1).unwrap();
```

Path routing is not supported for hot groups for now.

> [!IMPORTANT]
> The node indices passed to `hot_group.distance` refer to the nodes' indices within the slice passed to `create_hot_group`, not the indices of the nodes in the original contraction hierarchy.

## Saving and loading

Contraction hierarchies can be saved and loaded from disk:

```rust
let ch = ContractionHierarchy::new(&nodes, &edges, max_speed);

ch.save("poland.ch").unwrap();

let ch = ContractionHierarchy::load("poland.ch").unwrap();
```
