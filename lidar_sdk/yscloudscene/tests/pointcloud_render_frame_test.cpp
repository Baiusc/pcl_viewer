#include "business/pointcloud_parser.h"
#include "business/pointcloud_parser_imp.h"
#include "component/pointcloud/pointcloud_render_mailbox.h"

#include <array>
#include <barrier>
#include <cmath>
#include <cstring>
#include <limits>
#include <thread>

#include <QOpenGLContext>
#include <QtTest>

using namespace ys_scene;

namespace {
you_see::CartesianPoint MakePoint(float x, float y, float z)
{
    you_see::CartesianPoint point{};
    point.x.f = x;
    point.y.f = y;
    point.z.f = z;
    point.reflectivity = 128;
    return point;
}
} // namespace

class PointcloudRenderFrameTest : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase()
    {
        qRegisterMetaType<PointcloudRenderFramePtr>();
        colors_.fill(Rgb(10, 20, 30));
    }

    void validityRejectsMismatchedColors()
    {
        PointcloudRenderFrame frame;
        QVERIFY(!frame.IsValid());
        auto points = std::make_shared<PointcloudRenderFrame::Points>();
        frame.points = points;
        QVERIFY(frame.IsValid()); // Empty frames clear the display.
        points->push_back(MakePoint(1, 2, 3));
        QVERIFY(!frame.IsValid());
        frame.colors.emplace_back(10, 20, 30);
        QVERIFY(frame.IsValid());
        frame.frame_id = -1;
        QVERIFY(!frame.IsValid());
    }

    void cpuBuildPreservesInputAndResetsBounds()
    {
        QVERIFY(QOpenGLContext::currentContext() == nullptr);
        PointcloudBuild builder(colors_);
        builder.SetDistanceThrehold({0, 100});
        auto source = std::make_shared<PointcloudRenderFrame::Points>();
        source->push_back(MakePoint(2, 3, 4));
        source->push_back(MakePoint(5, 7, 9));
        auto input = source;
        auto frame = builder.PointcloudGenFlat(input, 0);
        QVERIFY(frame && frame->IsValid());
        QCOMPARE(frame->bounds_min, QVector3D(2, 3, 4));
        QCOMPARE(frame->bounds_max, QVector3D(5, 7, 9));
        QVERIFY(qAbs(frame->bounds_diagonal - std::sqrt(50.0)) < 0.0001);
        QCOMPARE(source->size(), std::size_t(2));
        QVERIFY(frame->points.get() != source.get());

        auto smaller = std::make_shared<PointcloudRenderFrame::Points>();
        smaller->push_back(MakePoint(3, 4, 5));
        auto second = builder.PointcloudGenFlat(smaller, 1);
        QVERIFY(second);
        QCOMPARE(second->bounds_min, QVector3D(3, 4, 5));
        QCOMPARE(second->bounds_max, QVector3D(3, 4, 5));
        QCOMPARE(second->bounds_diagonal, 0.0);
        QCOMPARE(frame->bounds_max, QVector3D(5, 7, 9));
        QVERIFY(QOpenGLContext::currentContext() == nullptr);
    }

    void structuredFrameAndAllMirrorColors()
    {
        PointcloudBuild builder(colors_);
        builder.SetDistanceThrehold({0, 100});
        builder.SetROI(ROISelectType::kROISet, PointCloudROIA{});
        builder.SetAllSidesDisplay(true);
        auto mapping = MappingMethod::kReflectivetyMap;
        builder.SetMappingMethod(mapping);
        auto points = std::make_shared<PointcloudRenderFrame::Points>();
        points->push_back(MakePoint(1, 2, 3));
        points->push_back(MakePoint(4, 5, 6));
        auto source = points;
        auto slot_counts = std::make_shared<std::vector<int>>(std::initializer_list<int>{0, 1});
        auto frame = builder.PointcloudGen(points, slot_counts, 0);
        QVERIFY(frame && frame->IsValid());
        QCOMPARE(frame->points->size(), std::size_t(1));
        QCOMPARE(frame->colors.size(), std::size_t(1));
        QCOMPARE(source->size(), std::size_t(2));
    }

    void emptyAndMalformedInputs()
    {
        PointcloudBuild builder(colors_);
        builder.SetDistanceThrehold({0, 100});
        CloudPureFramePtr missing;
        QVERIFY(!builder.PointcloudGenFlat(missing, 0));
        auto points = std::make_shared<PointcloudRenderFrame::Points>();
        auto empty = builder.PointcloudGenFlat(points, 0);
        QVERIFY(empty && empty->IsValid());
        QCOMPARE(empty->bounds_diagonal, 0.0);
        QVERIFY(!builder.PointcloudGenFlat(points, -1));
        auto slot_counts = std::make_shared<std::vector<int>>();
        QVERIFY(!builder.PointcloudGen(points, slot_counts, 0));
        slot_counts->assign({0, 1});
        points->push_back(MakePoint(1, 2, 3));
        QVERIFY(!builder.PointcloudGen(points, slot_counts, 0));
        points->front().z.f = std::numeric_limits<float>::infinity();
        QVERIFY(!builder.PointcloudGenFlat(points, 0));
    }

    void structuredInvalidPixelDoesNotDiscardFrame()
    {
        PointcloudBuild builder(colors_);
        builder.SetROI(ROISelectType::kROIReset, {});
        auto points = std::make_shared<PointcloudRenderFrame::Points>();
        points->push_back(MakePoint(5, 2, 1));
        points->push_back(MakePoint(5, 2, std::numeric_limits<float>::quiet_NaN()));
        points->push_back(MakePoint(5, 2, std::numeric_limits<float>::infinity()));
        auto slot_counts = std::make_shared<std::vector<int>>(std::initializer_list<int>{0});
        auto frame = builder.PointcloudGen(points, slot_counts, 0);
        QVERIFY(frame && frame->IsValid());
        QCOMPARE(frame->points->size(), std::size_t(1));
        QCOMPARE(frame->bounds_min, QVector3D(5, 2, 1));
    }

    void queuedFrameOutlivesWorkerInput()
    {
        PointcloudParserImp processor(colors_);
        processor.SetDistanThrehold({0, 100});
        PointcloudRenderFramePtr received;
        connect(&processor, &PointcloudParserImp::RenderFrameReady, this,
                [&received](PointcloudRenderFramePtr frame) { received = std::move(frame); },
                Qt::QueuedConnection);
        std::thread worker([&processor] {
            auto input = std::make_shared<PointcloudRenderFrame::Points>();
            input->push_back(MakePoint(1, 2, 3));
            processor.PointcloudProcessorFlat(input, {0, ""}, 2);
        });
        worker.join();
        QVERIFY(!received);
        QTRY_VERIFY(received != nullptr);
        QVERIFY(received->IsValid());
        QCOMPARE(received->points->front().z.f, 3.0f);
        QCOMPARE(received->frame_id, 2);
    }

    void parserOwnsAndStopsWorkers()
    {
        QObject parent;
        PointcloudParser parser(&parent);
        QCOMPARE(parser.parent(), &parent);
        parser.Stop();
    }

    void builderSnapshotOwnsPaletteAndSettings()
    {
        std::array<Rgb, max_steps_> palette;
        palette.fill(Rgb(255, 0, 0));
        PointcloudBuild settings(palette);
        settings.SetDistanceThrehold({0, 100});
        auto snapshot = settings;
        palette.fill(Rgb(0, 0, 255));
        settings.SetColorScale(palette);
        settings.SetDistanceThrehold({0, 5});
        auto source = std::make_shared<PointcloudRenderFrame::Points>();
        source->push_back(MakePoint(1, 0, 0));
        source->push_back(MakePoint(10, 0, 0));
        auto input = source;
        const auto old_frame = snapshot.PointcloudGenFlat(input, 0);
        input = source;
        const auto new_frame = settings.PointcloudGenFlat(input, 1);
        QVERIFY(old_frame && new_frame);
        QCOMPARE(old_frame->points->size(), std::size_t(2));
        QCOMPARE(new_frame->points->size(), std::size_t(1));
        QCOMPARE(old_frame->colors.front().r, 255);
        QCOMPARE(old_frame->colors.front().b, 0);
        QCOMPARE(new_frame->colors.front().r, 0);
        QCOMPARE(new_frame->colors.front().b, 255);
        QCOMPARE(source->size(), std::size_t(2));
    }

    void mailboxCoalescesAndRejectsLateFramesAfterTake()
    {
        PointcloudRenderMailbox mailbox;
        QSignalSpy notifications(&mailbox, &PointcloudRenderMailbox::FramesAvailable);
        const auto make_frame = [] {
            auto frame = std::make_shared<PointcloudRenderFrame>();
            frame->points = std::make_shared<PointcloudRenderFrame::Points>();
            frame->sequence = NextPointcloudRenderSequence();
            return frame;
        };
        auto old = make_frame();
        auto latest = make_frame();
        mailbox.Publish(old);
        const std::weak_ptr<const PointcloudRenderFrame> retired = old;
        mailbox.Publish(latest);
        mailbox.Publish(old);
        QCOMPARE(notifications.count(), 1);
        auto taken = mailbox.Take();
        QCOMPARE(taken[0].get(), latest.get());
        old.reset();
        QVERIFY(retired.expired());
        mailbox.Publish(latest);
        QVERIFY(!mailbox.Take()[0]);
        QCOMPARE(notifications.count(), 1);
        mailbox.Publish(make_frame());
        QCOMPARE(notifications.count(), 2);
    }

    void mailboxKeepsEachMirrorAndHonorsModeAndFreezeBoundaries()
    {
        PointcloudRenderMailbox mailbox;
        const auto make_frame = [](int id) {
            auto frame = std::make_shared<PointcloudRenderFrame>();
            frame->points = std::make_shared<PointcloudRenderFrame::Points>();
            frame->frame_id = id;
            frame->sequence = NextPointcloudRenderSequence();
            return frame;
        };
        auto before_mode_switch = make_frame(0);
        mailbox.SetAllMirrors(true);
        mailbox.Publish(before_mode_switch);
        QVERIFY(!mailbox.Take()[0]);
        std::array<PointcloudRenderFramePtr, 4> expected;
        for (int i = 0; i < 4; ++i) {
            expected[i] = make_frame(i);
        }
        // Completion order differs from submission order across different faces.
        for (int i = 3; i >= 0; --i) {
            mailbox.Publish(expected[i]);
        }
        const auto taken = mailbox.Take();
        for (int i = 0; i < 4; ++i) {
            QCOMPARE(taken[i].get(), expected[i].get());
        }
        mailbox.SetFrozen(true);
        auto during_freeze = make_frame(0);
        mailbox.Publish(during_freeze);
        QVERIFY(!mailbox.Take()[0]);
        mailbox.SetFrozen(false);
        mailbox.Publish(during_freeze);
        QVERIFY(!mailbox.Take()[0]);
        mailbox.Publish(make_frame(0));
        QVERIFY(mailbox.Take()[0]);
        mailbox.Close();
        mailbox.Publish(make_frame(1));
        QVERIFY(!mailbox.Take()[1]);
    }

    void taskSequencePrecedesExecutionAcrossProcessors()
    {
        PointcloudParserImp first(colors_);
        PointcloudParserImp second(colors_);
        first.SetDistanThrehold({0, 100});
        second.SetDistanThrehold({0, 100});
        auto mailbox = std::make_shared<PointcloudRenderMailbox>();
        const auto publish = [mailbox](PointcloudRenderFramePtr frame) { mailbox->Publish(std::move(frame)); };
        connect(&first, &PointcloudParserImp::RenderFrameReady, &first, publish, Qt::DirectConnection);
        connect(&second, &PointcloudParserImp::RenderFrameReady, &second, publish, Qt::DirectConnection);
        auto input = std::make_shared<PointcloudRenderFrame::Points>(1, MakePoint(1, 0, 0));
        PointcloudTaskFlat older(input, &first, {0, ""}, 3);
        PointcloudTaskFlat newer(input, &second, {0, ""}, 0);
        newer.run();
        const auto latest = mailbox->Take()[0];
        QVERIFY(latest);
        QCOMPARE(latest->frame_id, 0);
        older.run();
        QVERIFY(!mailbox->Take()[0]);
    }

    void concurrentMailboxPublishRetainsNewestSubmission()
    {
        PointcloudRenderMailbox mailbox;
        std::array<PointcloudRenderFramePtr, 200> frames;
        for (auto& frame : frames) {
            auto next = std::make_shared<PointcloudRenderFrame>();
            next->points = std::make_shared<PointcloudRenderFrame::Points>();
            next->sequence = NextPointcloudRenderSequence();
            frame = std::move(next);
        }
        std::barrier start(3);
        const auto publish = [&](int offset) {
            start.arrive_and_wait();
            for (int i = offset; i < static_cast<int>(frames.size()); i += 2) {
                mailbox.Publish(frames[i]);
            }
        };
        std::jthread first(publish, 0);
        std::jthread second(publish, 1);
        start.arrive_and_wait();
        first.join();
        second.join();
        QCOMPARE(mailbox.Take()[0].get(), frames.back().get());
    }

    void colorsDoNotDependOnPreviousFrame()
    {
        std::array<Rgb, max_steps_> palette;
        for (std::size_t i = 0; i < palette.size(); ++i) {
            palette[i] = Rgb(static_cast<ColorCompType>(i * 255 / (palette.size() - 1)), 0, 0);
        }
        PointcloudBuild builder(palette);
        auto mapping = MappingMethod::kReflectivetyMap;
        builder.SetMappingMethod(mapping);
        builder.SetDistanceThrehold({0, 100});
        builder.SetROI(ROISelectType::kROIReset, {});
        const auto slot_counts = std::make_shared<std::vector<int>>(1, 0);
        for (const bool structured : {false, true}) {
            auto render = [&](uint8_t reflectivity) {
                auto input = std::make_shared<PointcloudRenderFrame::Points>(1, MakePoint(1, 0, 0));
                input->front().reflectivity = reflectivity;
                return structured ? builder.PointcloudGen(input, slot_counts, 0) : builder.PointcloudGenFlat(input, 0);
            };
            const auto first = render(200);
            QVERIFY(first && first->IsValid());
            QVERIFY(render(10));
            const auto repeated = render(200);
            QVERIFY(repeated && repeated->IsValid());
            QCOMPARE(first->colors.front().r, repeated->colors.front().r);
            // 255 - 200 = 55: the existing quantized LUT lookup yields approximately 55/255.
            QVERIFY(first->colors.front().r >= 53 && first->colors.front().r <= 55);
        }
    }

    void concurrentPaletteUpdatesKeepEachFrameConsistent()
    {
        std::array<Rgb, max_steps_> red;
        std::array<Rgb, max_steps_> blue;
        red.fill(Rgb(255, 0, 0));
        blue.fill(Rgb(0, 0, 255));
        PointcloudParserImp processor(red);
        processor.SetDistanThrehold({0, 100});
        processor.SetROI(ROISelectType::kROIReset, {});
        std::vector<PointcloudRenderFramePtr> received;
        connect(&processor, &PointcloudParserImp::RenderFrameReady, this,
            [&received](PointcloudRenderFramePtr frame) { received.push_back(std::move(frame)); },
            Qt::QueuedConnection);
        constexpr int frames_per_worker = 20;
        std::barrier start(3);
        auto run = [&](bool structured) {
            start.arrive_and_wait();
            for (int i = 0; i < frames_per_worker; ++i) {
                auto input = std::make_shared<PointcloudRenderFrame::Points>(4096, MakePoint(1, 2, 3));
                if (structured) {
                    auto slot_counts = std::make_shared<std::vector<int>>(64, 0);
                    processor.PointcloudProcessor(input, slot_counts, {0, ""}, i);
                }
                else {
                    processor.PointcloudProcessorFlat(input, {0, ""}, i);
                }
            }
        };
        std::jthread first(run, false);
        std::jthread second(run, true);
        start.arrive_and_wait();
        for (int i = 0; i < 200; ++i) {
            processor.SetColorScale(i % 2 == 0 ? blue : red);
            processor.SetDistanThrehold({0, i % 2 == 0 ? 50 : 100});
        }
        first.join();
        second.join();
        QTRY_COMPARE(received.size(), std::size_t(2 * frames_per_worker));
        for (const auto& frame : received) {
            QVERIFY(frame && frame->IsValid());
            QCOMPARE(frame->points->size(), std::size_t(4096));
            const auto expected = frame->colors.front();
            QVERIFY((expected.r == 255 && expected.b == 0) || (expected.r == 0 && expected.b == 255));
            for (const auto color : frame->colors) {
                QCOMPARE(color.r, expected.r);
                QCOMPARE(color.g, 0);
                QCOMPARE(color.b, expected.b);
            }
        }
    }

    void gammaPacketsProduceOneCompleteRenderFrame()
    {
        PointcloudParser parser;
        parser.SetOptionalSlotNumber(static_cast<int>(OptionalSlotNumber::KSlot800));
        std::vector<PointcloudRenderFramePtr> received;
        for (auto* processor : parser.GetProcessors()) {
            processor->SetROI(ROISelectType::kROIReset, {});
            connect(processor, &PointcloudParserImp::RenderFrameReady, this,
                [&received](PointcloudRenderFramePtr frame) { received.push_back(std::move(frame)); },
                Qt::QueuedConnection);
        }

        constexpr int kPixels = 120;
        std::vector<uint8_t> packet_bytes(offsetof(you_see::PointCloudNetPacket, points) +
            kPixels * sizeof(you_see::CartesianPoint));
        auto* packet = reinterpret_cast<you_see::PointCloudNetPacket*>(packet_bytes.data());
        packet->total_slots = 800;
        for (int pixel = 0; pixel < kPixels; ++pixel) {
            const auto point = MakePoint(5, 2, static_cast<float>(pixel) / 10);
            std::memcpy(packet->points + pixel * sizeof(point), &point, sizeof(point));
        }
        // 接收从某个面的中间开始；下一面作为首个完整面。
        packet->frame_index = 3;
        packet->current_slot = 799;
        parser.PointcloudReceive2(0, packet, kPixels, nullptr);
        packet->frame_index = 0;
        for (int slot = 0; slot < 799; ++slot) {
            packet->current_slot = slot;
            parser.PointcloudReceive2(0, packet, kPixels, nullptr);
        }
        // 大量重复包不能补足缺失的第 800 个槽位。
        for (int repeat = 0; repeat < 10; ++repeat) {
            parser.PointcloudReceive2(0, packet, kPixels, nullptr);
        }
        QTest::qWait(60);
        QVERIFY(received.empty());

        packet->current_slot = 799;
        parser.PointcloudReceive2(0, packet, kPixels, nullptr);
        QTRY_COMPARE(received.size(), std::size_t(1));
        QVERIFY(received.front()->IsValid());
        QCOMPARE(received.front()->points->size(), std::size_t(96000));
        QCOMPARE(received.front()->bounds_max.z(), 11.9f);
        // 末包不能在完整帧发出后再产生一个超时残帧。
        QTest::qWait(350);
        QCOMPARE(received.size(), std::size_t(1));
        parser.Stop();
    }

private:
    std::array<Rgb, max_steps_> colors_{};
};

QTEST_GUILESS_MAIN(PointcloudRenderFrameTest)
#include "pointcloud_render_frame_test.moc"
