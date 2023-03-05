module DriverRank

using CSV
using DataFrames: DataFrame
using Transducers: MapCat, Map
using DataStructures: OrderedSet
using HypothesisTests: OneSampleZTest, pvalue
using Roots: find_zero
using Statistics: mean
import Optim
using Optim: optimize

struct TeamKey
    key::String
end

Base.@kwdef struct DriverMatchup{K}
    winner::K
    loser::K
end

Base.@kwdef struct DriverRankings{K}
    team_keys::OrderedSet{K}
    matchup_contributions::Matrix{Float64}
    expected_win_rate_func
end

function DriverRankings(matchups::Vector{DriverMatchup{K}}) where K
    team_keys =
        matchups |>
        MapCat(matchup -> (matchup.winner, matchup.loser)) |>
        OrderedSet{K}
    team_key_indecies =
        zip(team_keys, 1:length(team_keys)) |>
        Dict{TeamKey, Int}

    matchup_contributions = zeros(length(matchups), length(team_keys))
    for (i, matchup) in enumerate(matchups)
        contribution = view(matchup_contributions, i, :)

        winner_index = team_key_indecies[matchup.winner]
        loser_index = team_key_indecies[matchup.loser]

        contribution[winner_index] = 1
        contribution[loser_index] = -1
    end

    # Create a distribution that represents
    # how to translate player ranking point differences
    # into win rates
    point_difference = 100
    win_rate_at_point_difference = 0.9
    dist_std_dev = find_zero(
        x -> win_rate_at_point_difference - pvalue(OneSampleZTest(point_difference, x, 1), tail=:left),
        (0,Inf),
    )
    expected_win_rate_func(x) = pvalue(OneSampleZTest(x, dist_std_dev, 1), tail=:left)

    return DriverRankings{K}(;
        team_keys,
        matchup_contributions,
        expected_win_rate_func,
    )
end

num_teams(dr::DriverRankings) = length(dr.team_keys)

function objective_value(
    driver_rankings::DriverRankings,
    ranking_points::Vector{F},
) where F
    ranking_points_row = reshape(ranking_points, (1, num_teams(driver_rankings)))
    return objective_value(driver_rankings, ranking_points_row)
end

function objective_value(
    driver_rankings::DriverRankings,
    ranking_points::Matrix{F},
) where F
    average_ranking_point_value::F =
        mean(ranking_points)

    k = 100 / length(ranking_points) # magic number
    return -(k * log_likelihood(driver_rankings, ranking_points)) +
        (average_ranking_point_value^2)
end

function log_likelihood(
    driver_rankings::DriverRankings,
    ranking_points::Matrix{F},
) where F
    matchup_ranking_point_differentials =
        driver_rankings.matchup_contributions .* ranking_points |>
        x -> sum(x, dims=2)

    result::F =
        driver_rankings.expected_win_rate_func.(matchup_ranking_point_differentials) |>
        Map(log) |>
        sum
    return result
end

function rank()
    df = DataFrame(CSV.File("./data/2022_madtown.csv"))

    rank1 = "Rank 1 (best)"
    rank2 = "Rank 2"
    rank3 = "Rank 3 (worst)"
    rank_cols = [rank1, rank2, rank3]


    df[!, rank_cols] = TeamKey.(df[!, rank_cols])
    matchups =
        [
            (df[!, rank1], df[!, rank2]),
            (df[!, rank1], df[!, rank3]),
            (df[!, rank2], df[!, rank3]),
        ] |>
        MapCat(((winners, losers),) -> zip(winners, losers)) |>
        Map(((winner, loser),) -> DriverMatchup(; winner, loser)) |>
        collect

    driver_rankings = DriverRankings(matchups)

    # Optimize!
    x0 = zeros(num_teams(driver_rankings))
    res = optimize(x -> objective_value(driver_rankings, x), x0, Optim.LBFGS(), autodiff=:forward)

    ranking_points =
        DataFrame(
            :team=>driver_rankings.team_keys |> Map(x -> x.key) |> collect,
            :score=>Optim.minimizer(res),
        ) |>
        x -> sort!(x, [:score], rev=true)
    show(ranking_points, allrows=true)
end

export rank

end # module
