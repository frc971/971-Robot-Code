package scraping

type EventRanking struct {
	Rankings []Rank `json:"rankings"`
}

type Rank struct {
	MatchesPlayed int32     `json:"matches_played"`
	QualAverage   int32     `json:"qual_average"`
	ExtraStats    []float64 `json:"extra_stats"`
	SortOrders    []float64 `json:"sort_orders"`
	Records       Record    `json:"record"`
	Rank          int32     `json:"rank"`
	Dq            int32     `json:"dq"`
	TeamKey       string    `json:"team_key"`
}

type Record struct {
	Losses int32 `json:"losses"`
	Wins   int32 `json:"wins"`
	Ties   int32 `json:"ties"`
}

// Match holds the TBA data for a given match
type Match struct {
	Key             string
	CompLevel       string          `json:"comp_level"`
	SetNumber       int             `json:"set_number"`
	MatchNumber     int             `json:"match_number"`
	Alliances       Alliances       `json:"alliances"`
	WinningAlliance string          `json:"winning_alliance"`
	EventKey        string          `json:"event_key"`
	Time            int             `json:"time"`
	PredictedTime   int             `json:"predicted_time"`
	ActualTime      int             `json:"actual_time"`
	PostResultTime  int             `json:"post_result_time"`
	ScoreBreakdowns ScoreBreakdowns `json:"score_breakdowns"`
}

// Holds score breakdowns for each alliance
type ScoreBreakdowns struct {
	Blue ScoreBreakdownAlliance `json:"blue"`
	Red  ScoreBreakdownAlliance `json:"red"`
}

// Stores the actual data for the breakdown
type ScoreBreakdownAlliance struct {
	TaxiRobot1    string `json:"taxiRobot1"`
	EndgameRobot1 string `json:"endgameRobot1"`
	TaxiRobot2    string `json:"taxiRobot2"`
	EndgameRobot2 string `json:"endgameRobot2"`
	TaxiRobot3    string `json:"taxiRobot3"`
	EndgameRobot3 string `json:"endgameRobot3"`

	AutoCargoLowerNear      int  `json:"autoCargoLowerNear"`
	AutoCargoLowerFar       int  `json:"autoCargoLowerFar"`
	AutoCargoLowerBlue      int  `json:"autoCargoLowerBlue"`
	AutoCargoLowerRed       int  `json:"autoCargoLowerRed"`
	AutoCargoUpperNear      int  `json:"autoCargoUpperNear"`
	AutoCargoUpperFar       int  `json:"autoCargoUpperFar"`
	AutoCargoUpperBlue      int  `json:"autoCargoUpperBlue"`
	AutoCargoUpperRed       int  `json:"autoCargoUpperRed"`
	AutoCargoTotal          int  `json:"autoCargoTotal"`
	TeleOpCargoLowerNear    int  `json:"teleopCargoLowerNear"`
	TeleOpCargoLowerFar     int  `json:"teleopCargoLowerFar"`
	TeleOpCargoLowerBlue    int  `json:"teleopCargoLowerBlue"`
	TeleOpCargoLowerRed     int  `json:"teleopCargoLowerRed"`
	TeleOpCargoUpperNear    int  `json:"teleopCargoUpperNear"`
	TeleOpCargoUpperFar     int  `json:"teleopCargoUpperFar"`
	TeleOpCargoUpperBlue    int  `json:"teleopCargoUpperBlue"`
	TeleOpCargoUpperRed     int  `json:"teleopCargoUpperRed"`
	TeleopCargoTotal        int  `json:"teleopCargoTotal"`
	MatchCargoTotal         int  `json:"matchCargoTotal"`
	AutoTaxiPoints          int  `json:"autoTaxiPoints"`
	AutoCargoPoints         int  `json:"autoCargoPoints"`
	AutoPoints              int  `json:"autoPoints"`
	QuintetAchieved         bool `json:"quintetAchieved"`
	TeleOpCargoPoints       int  `json:"teleopCargoPoints"`
	EndgamePoints           int  `json:"endgamePoints"`
	TeleOpPoints            int  `json:"teleopPoints"`
	CargoBonusRankingPoint  bool `json:"cargoBonusRankingPoint"`
	HangerBonusRankingPoint bool `json:"hangarBonusRankingPoint"`
	FoulCount               bool `json:"foulCount"`
	TechFoulCount           int  `json:"techFoulCount"`
	AdjustPoints            int  `json:"adjustPoints"`
	FoulPoints              int  `json:"foulPoints"`
	RankingPoints           int  `json:"rp"`
	TotalPoints             int  `json:"totalPoints"`
}

// Alliances holds the two alliances for a match
type Alliances struct {
	Red  Alliance `json:"red"`
	Blue Alliance `json:"blue"`
}

// Alliance holds the info for the alliance
type Alliance struct {
	Score             int      `json:"score"`
	TeamKeys          []string `json:"team_keys"`
	SurrogateTeamKeys []string `json:"surrogate_team_keys"`
	DqTeamKeys        []string `json:"dq_team_keys"`
}
