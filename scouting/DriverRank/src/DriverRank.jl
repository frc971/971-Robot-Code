module DriverRank

using GoogleSheets: sheets_client, Spreadsheet, CellRange, get, AUTH_SCOPE_READONLY
using CSV
using DataFrames: DataFrame
using Transducers: Cat, MapCat, Map
using DataStructures: OrderedSet
using HypothesisTests: OneSampleZTest, pvalue
using Roots: find_zero
using Statistics: mean
import Optim
using Optim: optimize
using BlackBoxOptim: bboptimize, best_candidate, best_fitness
# using PlotlyJS
using Plots: scatter, hline!, plotlyjs, savefig, plotly
import PlotlyBase: to_html

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
    # client = sheets_client(AUTH_SCOPE_READONLY)
    # # spreadsheet_id = "13Cit7WrUxWz79iYVnoMoPc56W7H_cfr92jyT67tb2Xo"
    # spreadsheet_id = "1q-Cl2aW4IkHk8Vcfd7OuFt0g4o3itn4SXgBi8Z1b7UE"
    # range_name = "Form Responses 1"

    # sheet = Spreadsheet(spreadsheet_id)
    # range = CellRange(sheet, range_name)
    # result = get(client, range).values

    # # Filter empty rows
    # is_not_empty =  result[:, 1] .!= ""
    # result = result[is_not_empty, :]
    # df = DataFrame(TeamKey.(result[2:end, :]), result[1, :])

    df = DataFrame(CSV.File("./data/2022_madtown.csv"))

    # rank1 = "Rank 1 (best)"
    # rank2 = "Rank 2"
    # rank3 = "Rank 3"
    # rank4 = "Rank 4"
    # rank5 = "Rank 5"
    # rank6 = "Rank 6 (worst)"
    # matchups =
    #     [
    #         (df[!, rank1], df[!, rank2]),
    #         (df[!, rank1], df[!, rank3]),
    #         (df[!, rank1], df[!, rank4]),
    #         (df[!, rank1], df[!, rank5]),
    #         (df[!, rank1], df[!, rank6]),
    #         (df[!, rank2], df[!, rank3]),
    #         (df[!, rank2], df[!, rank4]),
    #         (df[!, rank2], df[!, rank5]),
    #         (df[!, rank2], df[!, rank6]),
    #         (df[!, rank3], df[!, rank4]),
    #         (df[!, rank3], df[!, rank5]),
    #         (df[!, rank3], df[!, rank6]),
    #         (df[!, rank4], df[!, rank5]),
    #         (df[!, rank4], df[!, rank6]),
    #         (df[!, rank5], df[!, rank6]),
    #     ] |>
    #     MapCat(((winners, losers),) -> zip(winners, losers)) |>
    #     Map(((winner, loser),) -> DriverMatchup(; winner, loser)) |>
    #     collect

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

    plotly()
    idx = 1:length(ranking_points.team)
    plt = scatter(
        idx, ranking_points.score,
        title="Driver Ranking",
        xlabel="Team Number",
        xticks=(idx, ranking_points.team),
        xrotation=90,
        ylabel="Score",
        legend=false,
    )
    hline!(plt, [0.])

    savefig(plt, "./driver_ranking.html")
    # open("./driver_ranking.html", "w") do io
    #     PlotlyBase.to_html(io, plt)
    # end

    return plt
end

export rank

end # module
